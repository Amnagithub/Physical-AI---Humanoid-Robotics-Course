import asyncio
import logging
from typing import List, Dict, Any, Optional
from pathlib import Path
import markdown
import re
from .qdrant_service import QdrantService
from ..models.database import TextbookContent
from ..database import get_sync_db
from sqlalchemy.orm import Session

logger = logging.getLogger(__name__)

class ContentIngestionService:
    def __init__(self, qdrant_service: QdrantService):
        self.qdrant_service = qdrant_service
        self.embedding_service = None  # Will be set later

    def set_embedding_service(self, embedding_service):
        """Set the embedding service after initialization to avoid circular dependencies"""
        self.embedding_service = embedding_service

    def _extract_sections(self, content: str, section_type: str = "chapters") -> List[Dict[str, str]]:
        """
        Extract content sections based on markdown headers or other patterns.
        This is a simplified implementation - in a real system you might have more sophisticated parsing.
        """
        sections = []

        # For markdown content, try to identify sections by headers
        lines = content.split('\n')
        current_section = ""
        current_content = ""

        for line in lines:
            # Check if line is a header (starts with #)
            if line.strip().startswith('#'):
                # Save the previous section if it exists
                if current_section and current_content:
                    sections.append({
                        'section': current_section.strip(),
                        'content': current_content.strip()
                    })

                # Start new section
                header_level = len(line) - len(line.lstrip('#'))
                section_title = line.strip('# ').strip()
                current_section = f"{section_type} {header_level}.{section_title}"
                current_content = ""
            else:
                current_content += line + '\n'

        # Add the last section
        if current_section and current_content:
            sections.append({
                'section': current_section.strip(),
                'content': current_content.strip()
            })

        return sections

    async def _process_and_store_content(self, content_id: str, section: str, content: str) -> bool:
        """Process content, generate embedding, and store in both Qdrant and database"""
        try:
            # Generate embedding for the content
            embedding = await self.embedding_service.generate_embedding(content)

            if not embedding:
                logger.error(f"Failed to generate embedding for content {content_id}")
                return False

            # Store in Qdrant
            qdrant_success = self.qdrant_service.add_textbook_content(
                content_id=content_id,
                content=content,
                section=section,
                embedding=embedding
            )

            if not qdrant_success:
                logger.error(f"Failed to store content {content_id} in Qdrant")
                return False

            # Store in database (for metadata and reference)
            # Use synchronous database operations for this task
            for db in get_sync_db():
                try:
                    # Check if content already exists
                    existing_content = db.query(TextbookContent).filter(TextbookContent.id == content_id).first()

                    if existing_content:
                        # Update existing content
                        existing_content.section = section
                        existing_content.content = content
                        db.commit()
                    else:
                        # Create new content
                        textbook_content = TextbookContent(
                            id=content_id,
                            section=section,
                            content=content
                        )
                        db.add(textbook_content)
                        db.commit()
                except Exception as e:
                    db.rollback()
                    logger.error(f"Database error when storing content {content_id}: {e}")
                    return False
                finally:
                    db.close()

            return True
        except Exception as e:
            logger.error(f"Error processing and storing content {content_id}: {e}")
            return False

    async def ingest_from_file(self, file_path: str) -> bool:
        """Ingest textbook content from a file"""
        try:
            file_path = Path(file_path)

            if not file_path.exists():
                logger.error(f"File does not exist: {file_path}")
                return False

            # Read the file
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract sections from the content
            sections = self._extract_sections(content, "Chapter")

            # Process each section
            successful_ingests = 0
            for i, section_data in enumerate(sections):
                section_title = section_data['section']
                section_content = section_data['content']

                # Generate a unique ID for this content section
                content_id = f"section_{i:04d}_{hash(section_title) % 10000}"

                # Process and store the content
                success = await self._process_and_store_content(
                    content_id, section_title, section_content
                )

                if success:
                    successful_ingests += 1
                    logger.info(f"Successfully ingested section: {section_title}")
                else:
                    logger.error(f"Failed to ingest section: {section_title}")

            logger.info(f"Content ingestion completed. {successful_ingests}/{len(sections)} sections ingested successfully.")
            return successful_ingests > 0

        except Exception as e:
            logger.error(f"Error ingesting content from file {file_path}: {e}")
            return False

    async def ingest_from_text(self, text: str, source_name: str = "manual_input") -> bool:
        """Ingest textbook content from a text string"""
        try:
            # Extract sections from the text
            sections = self._extract_sections(text, source_name)

            # Process each section
            successful_ingests = 0
            for i, section_data in enumerate(sections):
                section_title = section_data['section']
                section_content = section_data['content']

                # Generate a unique ID for this content section
                content_id = f"{source_name}_section_{i:04d}_{hash(section_title) % 10000}"

                # Process and store the content
                success = await self._process_and_store_content(
                    content_id, section_title, section_content
                )

                if success:
                    successful_ingests += 1
                    logger.info(f"Successfully ingested section: {section_title}")
                else:
                    logger.error(f"Failed to ingest section: {section_title}")

            logger.info(f"Text ingestion completed. {successful_ingests}/{len(sections)} sections ingested successfully.")
            return successful_ingests > 0

        except Exception as e:
            logger.error(f"Error ingesting content from text: {e}")
            return False

    async def bulk_ingest_from_directory(self, directory_path: str, file_extensions: List[str] = None) -> Dict[str, Any]:
        """Ingest all compatible files from a directory"""
        if file_extensions is None:
            file_extensions = ['.md', '.txt', '.markdown']

        directory = Path(directory_path)
        if not directory.exists() or not directory.is_dir():
            logger.error(f"Directory does not exist: {directory_path}")
            return {"success": False, "message": "Directory does not exist"}

        files = []
        for ext in file_extensions:
            files.extend(directory.glob(f"*{ext}"))

        results = {
            "total_files": len(files),
            "successful": 0,
            "failed": 0,
            "details": []
        }

        for file_path in files:
            success = await self.ingest_from_file(str(file_path))
            if success:
                results["successful"] += 1
            else:
                results["failed"] += 1

            results["details"].append({
                "file": str(file_path),
                "success": success
            })

        logger.info(f"Bulk ingestion completed: {results['successful']} successful, {results['failed']} failed")
        return results