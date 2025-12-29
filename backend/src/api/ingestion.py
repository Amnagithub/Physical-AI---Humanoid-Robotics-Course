from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Dict, List, Any, Optional
import asyncio

from ..services.content_ingestion_service import ContentIngestionService

router = APIRouter()

# Initialize the ingestion service
ingestion_service = ContentIngestionService()


class IngestRequest(BaseModel):
    """Request model for ingesting book content"""
    book_title: str
    content: Dict[str, str]  # Section name -> content
    metadata: Optional[Dict[str, Any]] = None


class IngestSingleRequest(BaseModel):
    """Request model for ingesting single content"""
    content: str
    metadata: Dict[str, Any]


class ChunkConfig(BaseModel):
    """Optional chunking configuration"""
    chunk_size: int = 512
    overlap: int = 50


@router.post("/content/ingest")
async def ingest_book_content(request: IngestRequest, chunk_config: Optional[ChunkConfig] = None):
    """
    Ingest book content into the vector store for RAG retrieval.

    This endpoint chunks the content, generates embeddings, and stores them in Qdrant.
    """
    try:
        # Modify chunk size if provided
        if chunk_config:
            ingestion_service.chunk_size = chunk_config.chunk_size
            ingestion_service.overlap = chunk_config.overlap

        # Run the async ingestion
        point_ids = await asyncio.to_thread(
            ingestion_service.ingest_book_content,
            book_title=request.book_title,
            book_content=request.content,
            metadata=request.metadata
        )

        return {
            "status": "success",
            "message": f"Successfully ingested {len(point_ids)} chunks",
            "book_title": request.book_title,
            "chunks_ingested": len(point_ids)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to ingest content: {str(e)}")


@router.post("/content/ingest-single")
async def ingest_single_content(request: IngestSingleRequest):
    """
    Ingest a single piece of content into the vector store.
    """
    try:
        point_id = await asyncio.to_thread(
            ingestion_service.ingest_single_content,
            content=request.content,
            metadata=request.metadata
        )

        return {
            "status": "success",
            "message": "Content ingested successfully",
            "point_id": point_id
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to ingest content: {str(e)}")


@router.get("/content/stats")
async def get_content_stats():
    """
    Get statistics about the ingested content.
    """
    try:
        qdrant_service = ingestion_service.qdrant_service
        collection_info = qdrant_service.get_collection_info()

        return {
            "status": "success",
            "stats": collection_info
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get stats: {str(e)}")
