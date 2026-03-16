from fastapi import APIRouter
from pydantic import BaseModel
from openai import OpenAI
import os

router = APIRouter()


class BumpRequest(BaseModel):
    user_id: int
    context_summary: str


class BumpResponse(BaseModel):
    user_id: int
    draft_message: str


@router.post("/bump", response_model=BumpResponse)
async def bump(request: BumpRequest):
    client = OpenAI(
        api_key=os.environ["MOONSHOT_API_KEY"],
        base_url="https://api.moonshot.cn/v1",
    )

    response = client.chat.completions.create(
        model="moonshot-v1-8k",
        max_tokens=512,
        messages=[
            {
                "role": "system",
                "content": (
                    "You are a friendly, natural follow-up message writer. "
                    "Write a short, conversational follow-up message to re-engage a contact. "
                    "Do not be pushy or salesy. Sound human. Return only the message text, nothing else."
                ),
            },
            {
                "role": "user",
                "content": (
                    f"Write a natural follow-up message to re-engage this contact.\n\n"
                    f"Last conversation summary:\n{request.context_summary}"
                ),
            },
        ],
    )
    draft = response.choices[0].message.content.strip()

    return BumpResponse(user_id=request.user_id, draft_message=draft)
