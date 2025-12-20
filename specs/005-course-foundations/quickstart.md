# Quickstart Guide: Course Foundations

## Overview
This guide explains how to work with the course foundation documents that provide essential context before students begin Module 1 of the Physical AI & Humanoid Robotics course.

## File Structure
```
docs/course/
├── why-physical-ai-matters.md
├── learning-outcomes.md
├── weekly-breakdown.md
├── hardware-requirements.md
└── lab-architecture.md
```

## Adding New Course Foundation Documents

1. Create the new Markdown file in `docs/course/`
2. Include proper YAML frontmatter:
   ```yaml
   ---
   sidebar_label: 'Document Title'
   title: 'Document Title'
   ---
   ```
3. Add the document to the sidebar configuration in `sidebars.js`:
   ```js
   {
     type: 'category',
     label: 'Course Foundations',
     items: [
       // ... existing items ...
       'course/new-document-name'
     ],
   }
   ```

## Referencing Course Foundations in Modules

To reference a course foundation document from a module:

```markdown
[Document Title](../../course/document-name.md)
```

## Best Practices

- Keep content technical and industry-oriented
- Ensure documents provide essential context for the course
- Maintain consistent formatting with other course materials
- Test all cross-references work properly
- Verify navigation works in the development server

## Development Workflow

1. Create or modify course foundation documents
2. Update sidebar configuration if adding new documents
3. Test in development server: `npm run start`
4. Verify all links and navigation work properly