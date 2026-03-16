from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List
from openai import OpenAI
import os

router = APIRouter()


class Message(BaseModel):
    sender: str
    text: str
    timestamp: str


class SummarizeRequest(BaseModel):
    messages: List[Message]
    platform: str  # "telegram" | "messenger"


class SummarizeResponse(BaseModel):
    summary: str


@router.post("/summarize", response_model=SummarizeResponse)
async def summarize(request: SummarizeRequest):
    if request.platform not in ("telegram", "messenger"):
        raise HTTPException(status_code=400, detail="platform must be 'telegram' or 'messenger'")

    if not request.messages:
        raise HTTPException(status_code=400, detail="messages list is empty")

    conversation = "\n".join(
        f"[{m.timestamp}] {m.sender}: {m.text}" for m in request.messages
    )

    client = OpenAI(
        api_key=os.environ["MOONSHOT_API_KEY"],
        base_url="https://api.moonshot.cn/v1",
    )

    response = client.chat.completions.create(
        model="moonshot-v1-8k",
        max_tokens=1024,
        messages=[
            {
                "role": "system",
                "content": (
                    "You are a concise conversation summarizer. "
                    "Summarize the key points, decisions, and action items from the conversation. "
                    "Be brief and clear."
                ),
            },
            {
                "role": "user",
                "content": f"Summarize this {request.platform} conversation:\n\n{conversation}",
            },
        ],
    )
    summary = response.choices[0].message.content

    return SummarizeResponse(summary=summary)
