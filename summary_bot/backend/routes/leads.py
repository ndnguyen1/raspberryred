from fastapi import APIRouter
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime, timezone
import os

router = APIRouter()


class TelegramContact(BaseModel):
    user_id: int
    username: Optional[str] = None
    last_message_date: str  # ISO 8601
    last_message_text: str


class StaleLeadsRequest(BaseModel):
    contacts: List[TelegramContact]
    stale_days: Optional[int] = None  # overrides LEAD_STALE_DAYS env var


class StaleContact(BaseModel):
    user_id: int
    username: Optional[str]
    last_message_date: str
    last_message_text: str
    days_since_reply: int


class StaleLeadsResponse(BaseModel):
    stale_leads: List[StaleContact]
    threshold_days: int


@router.post("/leads", response_model=StaleLeadsResponse)
async def get_stale_leads(request: StaleLeadsRequest):
    threshold = request.stale_days or int(os.environ.get("LEAD_STALE_DAYS", "3"))
    now = datetime.now(timezone.utc)
    stale = []

    for contact in request.contacts:
        last_msg_dt = datetime.fromisoformat(contact.last_message_date)
        if last_msg_dt.tzinfo is None:
            last_msg_dt = last_msg_dt.replace(tzinfo=timezone.utc)
        days_elapsed = (now - last_msg_dt).days

        if days_elapsed >= threshold:
            stale.append(
                StaleContact(
                    user_id=contact.user_id,
                    username=contact.username,
                    last_message_date=contact.last_message_date,
                    last_message_text=contact.last_message_text,
                    days_since_reply=days_elapsed,
                )
            )

    stale.sort(key=lambda c: c.days_since_reply, reverse=True)
    return StaleLeadsResponse(stale_leads=stale, threshold_days=threshold)
