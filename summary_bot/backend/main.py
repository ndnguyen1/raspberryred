from dotenv import load_dotenv
load_dotenv()

from fastapi import FastAPI
from routes.summarize import router as summarize_router
from routes.leads import router as leads_router
from routes.bump import router as bump_router

app = FastAPI(title="Summary Bot API", version="1.0.0")

app.include_router(summarize_router)
app.include_router(leads_router)
app.include_router(bump_router)


@app.get("/health")
async def health():
    return {"status": "ok"}
