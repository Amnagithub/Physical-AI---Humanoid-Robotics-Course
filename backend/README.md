# RAG Chatbot Backend

Backend service for the RAG Chatbot embedded in the Physical AI & Humanoid Robotics textbook.

## Overview

This service provides a Retrieval-Augmented Generation (RAG) chatbot that answers questions based solely on the textbook content, ensuring accurate and authoritative responses.

## Features

- Question answering using textbook content only
- Full-book and selected-text modes
- Session management for conversation history
- Source attribution for all responses
- Hallucination prevention

## Tech Stack

- FastAPI: Web framework
- OpenAI API: Language model integration
- Qdrant: Vector database for content retrieval
- PostgreSQL: Session and metadata storage

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Create environment file:
   ```bash
   cp .env.example .env
   # Update .env with your API keys and service URLs
   ```

3. Run the application:
   ```bash
   uvicorn src.main:app --reload
   ```

## API Documentation

The API documentation is available at `/docs` when the service is running.