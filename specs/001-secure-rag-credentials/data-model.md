# Data Model: Secure RAG Credentials Integration

## Credential Configuration Entity

### Fields
- **cohere_api_key**: String (required) - API key for Cohere service
- **qdrant_url**: String (required) - URL for Qdrant Cloud instance
- **qdrant_api_key**: String (required) - API key for Qdrant Cloud
- **neon_database_url**: String (required) - Connection string for Neon Postgres
- **app_env**: String (optional) - Application environment (default: "development")
- **debug**: Boolean (optional) - Debug mode flag (default: false)
- **log_level**: String (optional) - Logging level (default: "info")

### Validation Rules
- cohere_api_key must be a non-empty string of 32+ characters
- qdrant_url must be a valid HTTPS URL with port
- qdrant_api_key must be a non-empty string
- neon_database_url must be a valid PostgreSQL connection string

## Validation Result Entity

### Fields
- **is_valid**: Boolean - Overall validation status
- **missing_credentials**: List[String] - List of missing credential names
- **invalid_credentials**: List[String] - List of invalid credential names
- **connection_errors**: List[String] - List of connection error messages
- **timestamp**: DateTime - Time of validation

## Secure Environment Entity

### Fields
- **environment_name**: String - Name of the environment (dev/staging/prod)
- **credential_source**: String - Source of credentials (environment variables)
- **validation_strategy**: String - Strategy used for validation
- **security_level**: String - Security level applied

## Relationships
- Credential Configuration contains Validation Result (one-to-one)
- Credential Configuration belongs to Secure Environment (many-to-one)