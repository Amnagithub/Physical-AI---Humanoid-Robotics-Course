---
id: 2
title: Update Spec for Cohere RAG Chatbot
stage: spec
date: 2025-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-rag-chatbot
branch: 1-rag-chatbot
user: Claude
command: /sp.specify
labels: ["spec", "rag", "cohere", "chatbot"]
links:
  spec: specs/1-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-rag-chatbot/spec.md
  - specs/1-rag-chatbot/checklists/requirements.md
tests: []
---

# Prompt History Record

## Original Prompt
```
Project

Embedded RAG Chatbot for a Published Book

Goal

Build a Cohere-powered Retrieval-Augmented Generation (RAG) chatbot embedded in a published book, capable of answering questions only from book content or explicitly selected text.

Target Users

Readers, researchers, and educators requiring precise, text-grounded answers.

Required Stack

LLM & Embeddings: Cohere

API: FastAPI

Vector DB: Qdrant Cloud (Free Tier)

Relational DB: Neon Serverless Postgres

Prompt Governance: SpecifyKit Plus

Execution: Qween CLI

Disallowed: OpenAI APIs or SDKs

Credential Policy

All secrets must be injected via environment variables at runtime:

COHERE_API_KEY
QDRANT_ENDPOINT
QDRANT_API_KEY
QDRANT_CLUSTER_ID
NEON_DATABASE_URL


No credentials in prompts or source code.

Functional Scope (Build)

Book content ingestion & chunking

Cohere embeddings stored in Qdrant

Semantic retrieval (Top-K ≤ 5)

Context-bounded answer generation

Selected-text-only answer mode

Embeddable chatbot API

Out of Scope

External knowledge or web search

Multi-book reasoning

Model fine-tuning

Ethical or product comparisons

Behavioral Constraints

Answers must be strictly grounded in retrieved or selected text

No speculation or hallucination

Refuse if context is insufficient

Refusal message (exact):

"The provided text does not contain sufficient information to answer this question."

Quality Targets

Retrieval relevance ≥ 0.85

Latency ≤ 2s

Zero hallucination tolerance

Success Criteria

Accurate book-grounded answers

Selected text overrides global retrieval

Fully deployable via Qween CLI

Production-safe, reproducible behavior
```

## Response Summary
Updated the existing RAG chatbot specification to match the detailed requirements for a Cohere-powered embedded RAG chatbot. The updated spec now includes:

- Specific technology stack requirements (Cohere, Qdrant Cloud, Neon Postgres)
- Behavioral constraints with exact refusal message
- Functional requirements for book content ingestion, semantic retrieval, and context-bounded generation
- Quality targets for retrieval relevance (≥ 0.85) and latency (≤ 2s)
- Success criteria aligned with the project goals

Created a specification quality checklist to validate the completeness of the updated specification.

## Outcome
The specification for the embedded RAG chatbot has been updated to reflect all the detailed requirements including the specific technology stack, behavioral constraints, and quality targets. The spec is now ready for the planning phase.