<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0
Modified principles: N/A (initial creation)
Added sections: All sections
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ⚠ pending
- README.md ⚠ pending
Follow-up TODOs: None
-->

# AI-Native Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-Driven Development First
Spec-Kit Plus first, code second: All components must be defined in specifications before implementation begins; Claude Code generates code strictly from specs; Spec compliance is mandatory and deviations are prohibited.

### Accuracy and Zero Hallucination
The RAG chatbot must answer only from book content and never generate responses outside the provided context; All information retrieval must be grounded in the book's content; Answering from user-selected text is the only acceptable source for responses.

### Clear Educational Writing
Content must be written for a technical audience with clear explanations, examples, and educational value; All materials should follow textbook structure with chapters and sections; No plagiarism allowed; external sources must be properly cited when referenced.

### Reproducible and Modular Architecture
System architecture must be modular with clear separation of concerns; All components should be reproducible with documented setup and deployment procedures; Deterministic embeddings must be guaranteed for consistent retrieval.

### Technology Stack Standards
Docusaurus for book publishing with GitHub Pages deployment; OpenAI Agents/ChatKit with FastAPI for chatbot backend; Neon Serverless Postgres + Qdrant Cloud (Free Tier) for vector storage; Explicit API, DB schema, and environment specifications required.

### Compliance and Documentation
All API contracts and database schemas must be explicitly documented before implementation; Environment configurations must be deterministic and reproducible; Setup and deployment procedures must be thoroughly documented.

## Standards and Constraints

### Book Standards
- Written with Docusaurus framework for static site generation
- Structured as a textbook with organized chapters and sections
- Deployed on GitHub Pages for public accessibility
- No plagiarism permitted; all external sources must be properly cited

### Specification Standards
- All components must be defined in specs before any implementation work begins
- Claude Code must generate all code strictly from established specifications
- Spec compliance is mandatory with zero tolerance for deviations

### RAG Chatbot Standards
- Embedded seamlessly in the Docusaurus site for integrated user experience
- Built using OpenAI Agents/ChatKit and FastAPI for backend services
- Utilizes Neon Serverless Postgres + Qdrant Cloud (Free Tier) for vector storage
- Answers exclusively from book content with no external hallucinations
- Supports answering from user-selected text passages for precise context

## Development Workflow

### Pre-Implementation Requirements
- Explicit API contracts must be defined before any endpoint development
- Database schemas must be designed and documented before implementation
- Environment specifications must be completed before setup work begins
- All architectural decisions must be captured in specifications

### Quality Assurance
- Deterministic embeddings must be validated for consistent retrieval performance
- All components must have comprehensive test coverage before merging
- Integration tests must verify the connection between chatbot and book content
- Deployment procedures must be tested in staging before production release

### Documentation Standards
- All setup procedures must be documented with step-by-step instructions
- API documentation must be comprehensive and kept in sync with implementation
- Deployment guides must include troubleshooting sections
- Architecture decisions must be recorded in ADRs for future reference

## Governance

All development activities must comply with these constitutional principles. Deviations from spec-driven development, accuracy requirements, or technology stack decisions require formal constitution amendments. The constitution supersedes all other development practices and serves as the authoritative guide for all implementation decisions.

Every pull request must be reviewed for constitution compliance. Code reviews must verify adherence to spec-first principles, accuracy requirements, and architectural standards. All team members are responsible for maintaining constitutional compliance throughout the development lifecycle.

**Version**: 1.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17
