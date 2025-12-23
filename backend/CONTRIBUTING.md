# Contributing to RAG Chatbot Backend

Thank you for your interest in contributing to the RAG Chatbot Backend! This document outlines the process for contributing to this project.

## Development Setup

1. Clone the repository
2. Navigate to the `backend` directory
3. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
4. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
5. Copy the environment example file:
   ```bash
   cp .env.example .env
   ```
6. Update `.env` with your API keys and service URLs

## Project Structure

- `src/` - Main source code
- `tests/` - Test files
- `config/` - Configuration files
- `docs/` - Documentation

## Code Style

- Follow PEP 8 guidelines
- Use type hints for all function parameters and return values
- Write docstrings for all public functions and classes

## Testing

Before submitting a pull request, ensure all tests pass and add tests for new functionality.

## Pull Request Process

1. Create a feature branch
2. Make your changes
3. Add tests if applicable
4. Update documentation if needed
5. Submit a pull request with a clear description of your changes