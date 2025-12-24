# ADR 001: Multi-Provider LLM Support

## Status
Accepted

## Context
The RAG (Retrieval-Augmented Generation) chatbot for the Physical AI & Humanoid Robotics textbook needs to support multiple LLM providers to ensure:
- Vendor independence and flexibility
- Cost optimization by choosing the most cost-effective provider
- Reliability by having fallback options
- Performance optimization by selecting the best-performing model for specific tasks
- Avoiding vendor lock-in

The system should be able to switch between providers (OpenAI, Cohere, OpenRouter) via configuration without code changes.

## Decision
We will implement a multi-provider LLM support system with the following architecture:

1. **Provider Abstraction Layer**: Create a unified interface that abstracts the differences between LLM providers
2. **Configuration-Based Selection**: Use the `LLM_PROVIDER` environment variable to determine which provider to use at runtime
3. **Service Factory Pattern**: Implement a main service (OpenAIService) that dynamically instantiates the appropriate provider service based on configuration
4. **Unified API**: All provider services implement the same interface with methods like `generate_response()`, `generate_structured_response()`, etc.

### Implementation Details:

1. **OpenAIService** (`/src/services/openai_service.py`): Acts as the main service that routes requests to the appropriate provider based on the `LLM_PROVIDER` setting
2. **Provider Services**: Separate services for each provider:
   - CohereService (`/src/services/cohere_service.py`)
   - OpenRouterService (`/src/services/openrouter_service.py`)
   - Default OpenAI implementation
3. **EmbeddingService** (`/src/services/embedding_service.py`): Similarly abstracts embedding generation across providers
4. **Configuration** (`/src/config.py`): Defines the `LLM_PROVIDER` setting and validates required API keys

## Consequences

### Positive Consequences:
- **Flexibility**: Easy to switch between providers without code changes
- **Vendor Independence**: Reduced risk of vendor lock-in
- **Cost Optimization**: Ability to choose the most cost-effective provider
- **Reliability**: Fallback options if one provider experiences issues
- **Performance**: Ability to select the best-performing model for specific tasks
- **Maintainability**: Clean separation of concerns with provider-specific logic isolated in dedicated services

### Negative Consequences:
- **Complexity**: Additional complexity in the codebase to handle multiple providers
- **Maintenance Overhead**: Need to maintain provider-specific implementations
- **Feature Parity**: Potential for feature differences between providers
- **Testing**: More complex testing scenarios to cover all providers
- **API Limitations**: Must use the lowest common denominator of features across providers

## Technical Implementation

### Configuration
The system uses the following environment variables:
- `LLM_PROVIDER`: Set to "openai", "cohere", or "openrouter"
- Provider-specific API keys (e.g., `OPENAI_API_KEY`, `COHERE_API_KEY`, `OPENROUTER_API_KEY`)
- Provider-specific model names (e.g., `OPENAI_MODEL`, `COHERE_MODEL`, `OPENROUTER_MODEL`)

### Architecture Flow
```
Client Request
    ↓
RAGService (in rag_service.py)
    ↓
OpenAIService (abstraction layer)
    ↓
Provider Selection based on LLM_PROVIDER env var
    ↓
Actual LLM Provider (OpenAI/Cohere/OpenRouter)
```

### Error Handling
- Each provider service handles its own API errors
- Graceful fallback mechanisms
- Consistent error logging across providers

## Alternatives Considered

1. **Single Provider Approach**: Using only one LLM provider would be simpler but creates vendor lock-in and reduces flexibility
2. **Third-party Abstraction Libraries**: Using libraries like LangChain's LLM abstraction, but this would add external dependencies and potentially limit customization
3. **Plugin Architecture**: More complex plugin system for adding providers, but would be overkill for the current requirements

## Validation
- The implementation has been tested with Cohere provider successfully
- Configuration validation ensures required API keys are present
- Error handling is consistent across providers
- The system gracefully handles provider-specific limitations

## Links
- Implementation: `/src/services/openai_service.py`
- Provider Services: `/src/services/cohere_service.py`, `/src/services/openrouter_service.py`
- Configuration: `/src/config.py`
- Embedding Service: `/src/services/embedding_service.py`