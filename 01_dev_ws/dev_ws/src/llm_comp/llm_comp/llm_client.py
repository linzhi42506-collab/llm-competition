"""大模型调用客户端：同时支持本地 llama.cpp(llama-server) 与云端 OpenAI 兼容 API。

比赛讲解明确指出：今年的题目对本地小模型偏难，建议使用 DeepSeek / 智谱AI /
火山引擎 等云端 API。本模块把两种调用方式统一成 ``chat()`` 一个接口，
只需要改 ``config/llm_config.yaml`` 即可切换，代码零改动。
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from typing import Dict, List, Optional

# 常见云端服务商的 OpenAI 兼容接入点。api_key 统一从环境变量读取，
# 避免把密钥写进代码仓库（比赛提交源码时尤其重要）。
PROVIDER_PRESETS: Dict[str, Dict[str, str]] = {
    "local": {
        "base_url": "http://127.0.0.1:8081/v1",
        "model": "qwen2.5-coder",
        "api_key_env": "",
    },
    "deepseek": {
        "base_url": "https://api.deepseek.com/v1",
        "model": "deepseek-chat",
        "api_key_env": "DEEPSEEK_API_KEY",
    },
    "zhipu": {
        "base_url": "https://open.bigmodel.cn/api/paas/v4",
        "model": "glm-4-flash",
        "api_key_env": "ZHIPUAI_API_KEY",
    },
    "volcengine": {
        "base_url": "https://ark.cn-beijing.volces.com/api/v3",
        "model": "doubao-pro-32k",
        "api_key_env": "ARK_API_KEY",
    },
    "dashscope": {
        "base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
        "model": "qwen-plus",
        "api_key_env": "DASHSCOPE_API_KEY",
    },
    "siliconflow": {
        "base_url": "https://api.siliconflow.cn/v1",
        "model": "Qwen/Qwen2.5-7B-Instruct",
        "api_key_env": "SILICONFLOW_API_KEY",
    },
    "openai": {
        "base_url": "https://api.openai.com/v1",
        "model": "gpt-4o-mini",
        "api_key_env": "OPENAI_API_KEY",
    },
}


class LLMError(RuntimeError):
    """调用大模型失败。"""


class LLMClient:
    """极简 OpenAI ``/chat/completions`` 客户端。

    参数
    ----
    provider:  预设名（local/deepseek/zhipu/...）或 ``custom``
    base_url:  覆盖预设的接入点
    model:     覆盖预设的模型名
    api_key:   直接给出密钥（优先级最高）
    timeout:   单次请求超时（秒）
    """

    def __init__(
        self,
        provider: str = "local",
        base_url: Optional[str] = None,
        model: Optional[str] = None,
        api_key: Optional[str] = None,
        timeout: float = 30.0,
        temperature: float = 0.1,
        max_tokens: int = 512,
    ) -> None:
        preset = PROVIDER_PRESETS.get(provider, {})
        self.provider = provider
        self.base_url = (base_url or preset.get("base_url") or "").rstrip("/")
        self.model = model or preset.get("model") or "qwen2.5-coder"
        self.timeout = float(timeout)
        self.temperature = float(temperature)
        self.max_tokens = int(max_tokens)

        key = api_key
        if not key:
            env_name = preset.get("api_key_env", "")
            if env_name:
                key = os.environ.get(env_name, "")
        self.api_key = key or ""

        if not self.base_url:
            raise LLMError("base_url 为空，请检查 llm_config.yaml")

    # ------------------------------------------------------------------ #
    @property
    def is_cloud(self) -> bool:
        return self.provider not in ("local", "")

    def endpoint(self) -> str:
        return f"{self.base_url}/chat/completions"

    def chat(
        self,
        user_prompt: str,
        system_prompt: Optional[str] = None,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
    ) -> str:
        """发起一次对话补全，返回助手文本。失败抛 :class:`LLMError`。"""
        messages: List[Dict[str, str]] = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})
        messages.append({"role": "user", "content": user_prompt})

        payload = {
            "model": self.model,
            "messages": messages,
            "temperature": self.temperature if temperature is None else temperature,
            "max_tokens": self.max_tokens if max_tokens is None else max_tokens,
            "stream": False,
        }
        data = json.dumps(payload).encode("utf-8")

        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"

        req = urllib.request.Request(self.endpoint(), data=data, headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                body = resp.read().decode("utf-8", errors="replace")
        except urllib.error.HTTPError as exc:  # 4xx/5xx
            detail = exc.read().decode("utf-8", errors="replace")[:400]
            raise LLMError(f"HTTP {exc.code}: {detail}") from exc
        except Exception as exc:  # 网络不可达 / 超时
            raise LLMError(f"连接 {self.endpoint()} 失败: {exc}") from exc

        try:
            obj = json.loads(body)
            return obj["choices"][0]["message"]["content"]
        except Exception as exc:
            raise LLMError(f"响应解析失败: {body[:400]}") from exc

    # ------------------------------------------------------------------ #
    @classmethod
    def from_yaml(cls, cfg: Dict) -> "LLMClient":
        """从 ``llm_config.yaml`` 的 ``llm:`` 段构造客户端。"""
        cfg = dict(cfg or {})
        return cls(
            provider=cfg.get("provider", "local"),
            base_url=cfg.get("base_url"),
            model=cfg.get("model"),
            api_key=cfg.get("api_key") or None,
            timeout=cfg.get("timeout", 30.0),
            temperature=cfg.get("temperature", 0.1),
            max_tokens=cfg.get("max_tokens", 512),
        )

    def describe(self) -> str:
        key_state = "已配置" if self.api_key else ("不需要" if not self.is_cloud else "缺失")
        return f"provider={self.provider} model={self.model} url={self.endpoint()} api_key={key_state}"
