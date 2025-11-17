import torch
import torch.nn as nn
from transformers import AutoTokenizer, AutoModel
from typing import Union, List

class FrozenTextAdapter(nn.Module):
    """
    Fixed-LLM text encoder with two projection modes:
      - proj_mode='trainable': small trainable head (Linear->GELU->LayerNorm)
      - proj_mode='frozen': fixed orthonormal projection to out_dim
    pool ∈ {"cls", "mean", "masked_mean", "content_mean"}
    """
    def __init__(self, model_name: str = "mistralai/Mistral-7B-Instruct",
                 out_dim: int = 256, pool: str = "content_mean",
                 l2norm: bool = False,
                 proj_mode: str = "frozen",
                 output_mode: str = "pooled",      # {"pooled","tokens"}
                 max_tokens: int = 30              # only used when output_mode="tokens"
                 ):
        super().__init__()
        self.tok = AutoTokenizer.from_pretrained(model_name, use_fast=True)
        if self.tok.pad_token_id is None:
            self.tok.pad_token = self.tok.eos_token
        self.tok.padding_side = "left"  # safer for causal LMs

        # loaded model 
        self.lm  = AutoModel.from_pretrained(model_name)
        self.lm.eval()
        for p in self.lm.parameters():
            p.requires_grad_(False)

        self.out_dim   = out_dim
        self.pool      = pool
        self.l2norm    = l2norm
        self.proj_mode = proj_mode
        self.output_mode = output_mode
        self.max_tokens  = max_tokens

        H = self.lm.config.hidden_size
        if proj_mode == "trainable":
            self.proj = nn.Sequential(
                nn.Linear(H, out_dim, bias=False),
                nn.GELU(),
                nn.LayerNorm(out_dim, eps=1e-6),
            )
        elif proj_mode == "frozen":
            with torch.no_grad():
                gen = torch.Generator()
            # gen.manual_seed(4)  # fix seed   # this should not be needed 
            M = torch.randn(H, out_dim, generator=gen)
            Q, _ = torch.linalg.qr(M, mode="reduced")
            self.register_buffer("proj_frozen", Q)
        else:
            raise ValueError("proj_mode must be {'trainable','frozen'}")

    # unchanged pooler (assumes you already have this)
    def _pool_hidden(self, h, input_ids, attn_mask):
        # h: [B,T,H]
        if self.pool == "cls" and hasattr(self.tok, "cls_token_id") and self.tok.cls_token_id is not None:
            # first non-pad token (often CLS)
            return h[:, 0, :]
        elif self.pool in ("mean", "masked_mean", "content_mean"):
            if attn_mask is None:
                return h.mean(dim=1)
            w = attn_mask.float().unsqueeze(-1)              # [B,T,1]
            w = w / (w.sum(dim=1, keepdim=True) + 1e-9)
            return (h * w).sum(dim=1)                        # [B,H]
        else:
            return h[:, 0, :]

    def _project(self, x):
        # x: [..., H] -> [..., D]
        if self.proj_mode == "trainable":
            y = self.proj(x)
        else:
            y = x @ self.proj_frozen
        if self.l2norm:
            y = torch.nn.functional.normalize(y, dim=-1, eps=1e-8)
        return y

    def _pad_truncate_tokens(self, Z_tokens, attn_mask):
        # Z_tokens: [B,T,D], attn_mask: [B,T]
        B, T, D = Z_tokens.shape
        Tm = self.max_tokens
        if T >= Tm:
            return Z_tokens[:, :Tm, :]
        # pad with zeros; mask controls downstream use
        pad = Z_tokens.new_zeros((B, Tm - T, D))
        return torch.cat([Z_tokens, pad], dim=1)
    
    def save_adapter(self, path: str):
        """Save only the adapter-specific parameters, not the large LLM."""
        state = {}
        if self.proj_mode == "trainable":
            state["proj"] = self.proj.state_dict()
        elif self.proj_mode == "frozen":
            state["proj_frozen"] = self.proj_frozen
            # state["frozen_seed"] = self.frozen_seed
        torch.save(state, path)

    def load_adapter(self, path: str, map_location=None):
        """Load adapter weights without touching the frozen LLM."""
        state = torch.load(path, map_location=map_location)
        if "proj" in state:
            self.proj.load_state_dict(state["proj"])
        elif "proj_frozen" in state:
            self.register_buffer("proj_frozen", state["proj_frozen"])
            # self.frozen_seed = state.get("frozen_seed", 0)

    @torch.no_grad()
    def encode_tokens_(self, batch_text: Union[str, List[str]], device=None):
        dev = device or (self.proj[0].weight.device if self.proj_mode == "trainable" else self.proj_frozen.device)
        single = isinstance(batch_text, str)
        if single:
            batch_text = [batch_text]

        # tokens mode uses explicit max_length; pooled mode can keep default truncation
        if self.output_mode == "tokens":
            tok = self.tok(batch_text, return_tensors="pt", 
                           padding=True, truncation=True,
                           max_length=self.max_tokens).to(dev)
        else:  # pooled mode
            tok = self.tok(batch_text, return_tensors="pt", 
                           padding=True, truncation=True).to(dev)

        # "raw" output from the encoder 
        h = self.lm(**tok).last_hidden_state  # [B,T,H]

        if self.output_mode == "tokens":
            Z = self._project(h)              # [B,T,D]
            Z = self._pad_truncate_tokens(Z, tok.get("attention_mask", None))
            return Z.squeeze(0) if single else Z  # [Tm,D] or [B,Tm,D]
        else:  # pooled mode
            v = self._pool_hidden(h, tok["input_ids"], tok.get("attention_mask", None))  # [B,H]
            z = self._project(v)               # [B,D]
            return z.squeeze(0) if single else z

    # main forward (preserves old API)
    def forward(self, batch_text: Union[str, List[str]], inference: bool = True, device=None):
        
        if inference:
            with torch.no_grad():
                return self.encode_tokens_(batch_text, device=device)
            
        # training: grads only flow in projection if proj_mode='trainable'
        dev = device or (self.proj[0].weight.device if self.proj_mode == "trainable" else self.proj_frozen.device)
        single = isinstance(batch_text, str)
        if single:
            batch_text = [batch_text]

        if self.output_mode == "tokens":
            tok = self.tok(batch_text, return_tensors="pt", padding=True, truncation=True,
                           max_length=self.max_tokens).to(dev)
            h = self.lm(**tok).last_hidden_state            # [B,T,H] (no grad in LM anyway)
            Z = self._project(h)                            # [B,T,D]
            Z = self._pad_truncate_tokens(Z, tok.get("attention_mask", None))
            return Z.squeeze(0) if single else Z
        else:
            tok = self.tok(batch_text, return_tensors="pt", padding=True, truncation=True).to(dev)
            h  = self.lm(**tok).last_hidden_state
            v  = self._pool_hidden(h, tok["input_ids"], tok.get("attention_mask", None))  # [B,H]
            z  = self._project(v)                              # [B,D]
            return z.squeeze(0) if single else z
        

# class FrozenTextAdapter(nn.Module):
#     """
#     Fixed-LLM text encoder with two projection modes:
#       - proj_mode='trainable': small trainable head (Linear->GELU->LayerNorm)
#       - proj_mode='frozen': fixed orthonormal projection to out_dim
#     pool ∈ {"cls", "mean", "masked_mean", "content_mean"}
#     """
#     def __init__(self, model_name: str = "mistralai/Mistral-7B-Instruct",
#                  out_dim: int = 256, pool: str = "content_mean", 
#                  l2norm: bool = False,
#                  proj_mode: str = "trainable", 
#                  device: str | None = None):
        
#         super().__init__()
#         self.tok = AutoTokenizer.from_pretrained(model_name, use_fast=True)
#         if self.tok.pad_token_id is None:
#             self.tok.pad_token = self.tok.eos_token
#         self.tok.padding_side = "left"  # safer for causal LMs

#         self.lm  = AutoModel.from_pretrained(model_name)
#         self.lm.eval()
#         for p in self.lm.parameters():
#             p.requires_grad_(False)

#         self.out_dim = out_dim
#         self.pool = pool
#         self.l2norm = l2norm
#         self.proj_mode = proj_mode

#         H = self.lm.config.hidden_size
#         if proj_mode == "trainable":
#             self.proj = nn.Sequential(
#                 nn.Linear(H, out_dim),
#                 nn.GELU(),
#                 nn.LayerNorm(out_dim)
#             )
#             self.register_buffer("P_fixed", None)
#         elif proj_mode == "frozen":
#             with torch.no_grad():
#                 W = torch.randn(H, out_dim)
#                 q, _ = torch.linalg.qr(W, mode="reduced")  
#                 P = q[:, :out_dim]               # out_dim ≤ H ensured above
#             self.register_buffer("P_fixed", P)   # not trainable
#             self.proj = nn.Identity()
#         else:
#             raise ValueError("proj_mode must be 'trainable' or 'frozen'")

#         # Cache special token ids
#         self._special_ids = set()
#         for name in ("bos_token_id", "eos_token_id", "pad_token_id", "cls_token_id", "sep_token_id"):
#             tid = getattr(self.tok, name, None)
#             if tid is not None:
#                 self._special_ids.add(tid)

#         if device is not None:
#             self.to(device)

#     def _pool_hidden(self, h, input_ids, attn_mask):
#         # h:[B,T,H], input_ids/attn_mask:[B,T]
#         if self.pool == "cls":
#             v = h[:, 0]
#         elif self.pool == "mean":
#             v = h.mean(dim=1)
#         elif self.pool == "masked_mean":
#             m = attn_mask.float().unsqueeze(-1)
#             v = (h * m).sum(dim=1) / m.sum(dim=1).clamp_min(1e-8)
#         elif self.pool == "content_mean":
#             m = attn_mask.clone()
#             if len(self._special_ids) > 0:
#                 specials = torch.zeros_like(m, dtype=torch.bool)
#                 for tid in self._special_ids:
#                     specials |= (input_ids == tid)
#                 m = m.masked_fill(specials, 0)
#             m = m.float().unsqueeze(-1)
#             denom = m.sum(dim=1)
#             empty = (denom.squeeze(-1) < 0.5)
#             if empty.any():
#                 m_alt = attn_mask.float().unsqueeze(-1)
#                 v_alt = (h * m_alt).sum(dim=1) / m_alt.sum(dim=1).clamp_min(1e-8)
#             v = (h * m).sum(dim=1) / denom.clamp_min(1e-8)
#             if empty.any():
#                 v[empty] = v_alt[empty]
#         else:
#             raise ValueError(f"Unknown pool='{self.pool}'")
#         return v  # (optional L2 after projection)

#     def _project(self, v):
#         # v: [B,H] -> [B,out_dim]
#         if self.proj_mode == "trainable":
#             z = self.proj(v)
#         else:  # 'frozen'
#             z = v @ self.P_fixed  # [B,H]@[H,D] -> [B,D]
#         return nn.functional.normalize(z, dim=-1) if self.l2norm else z


#     def forward(self, texts: Union[str, List[str]], inference: bool = False) -> torch.Tensor:
#         """Encodes one string or a list of strings.
#         If inference=True: also disables grads on the small projection head."""
#         single = isinstance(texts, str)
#         batch_text = [texts] if single else texts

#         dev = next(self.lm.parameters()).device

#         # LLM is always frozen: compute with no_grad
#         with torch.no_grad():
#             tok = self.tok(batch_text, return_tensors="pt", padding=True, truncation=True).to(dev)
#             h   = self.lm(**tok).last_hidden_state                 # [B,T,H]
#             v   = self._pool_hidden(h, tok["input_ids"], tok["attention_mask"])  # [B,H]

#         if inference:
#             with torch.no_grad():
#                 z = self._project(v)                               # [B,D], no grads anywhere
#         else:
#             z = self._project(v)                                   # [B,D], grads only in proj if trainable

#         return z.squeeze(0) if single else z