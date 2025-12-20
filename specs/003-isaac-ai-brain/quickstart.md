# Quickstart: The AI-Robot Brain for Humanoid Robotics

**Feature**: Module 3 - Docusaurus-based book for Physical AI & Humanoid Robotics
**Date**: 2025-12-20

## Prerequisites

Before starting with Module 3, ensure you have:

- **Node.js** version 18 or higher
- **npm** or **yarn** package manager
- **Git** for version control
- A modern web browser for viewing the documentation
- Basic knowledge of robotics concepts and programming

## Installation

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Docusaurus

```bash
npm install @docusaurus/core@latest
```

Or if using yarn:

```bash
yarn add @docusaurus/core@latest
```

### 3. Create the Module Structure

```bash
mkdir -p docs/module-3
```

### 4. Install Additional Dependencies

```bash
npm install @docusaurus/module-type-aliases @docusaurus/preset-classic
```

## Setting Up the Docusaurus Site

### 1. Initialize Docusaurus (if not already done)

```bash
npx create-docusaurus@latest my-website classic
cd my-website
```

### 2. Create Module 3 Content Files

Create the following files in your `docs/module-3/` directory:

- `docs/module-3/index.md` - Module introduction
- `docs/module-3/chapter-1-simulation.md` - Simulation chapter
- `docs/module-3/chapter-2-vslam-navigation.md` - VSLAM and navigation chapter
- `docs/module-3/chapter-3-path-planning.md` - Path planning chapter

### 3. Update Configuration

Add the module to your `docusaurus.config.js`:

```javascript
module.exports = {
  // ... other config
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/your/repo/edit/main/',
        },
        // ... other config
      },
    ],
  ],
  // ... rest of config
};
```

### 4. Add to Sidebar

In your `sidebars.js`, add:

```javascript
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      items: [
        'module-3/index',
        'module-3/chapter-1-simulation',
        'module-3/chapter-2-vslam-navigation',
        'module-3/chapter-3-path-planning',
      ],
    },
    // ... other modules
  ],
};
```

## Running the Development Server

### 1. Start the Development Server

```bash
npm run start
```

Or with yarn:

```bash
yarn start
```

### 2. Access the Documentation

Open your browser to `http://localhost:3000` to view the documentation.

## Creating Your First Chapter

### 1. Chapter Template

Create a new chapter file with this template:

```markdown
---
title: Your Chapter Title
sidebar_label: Chapter X
---

# Chapter X: Your Chapter Title

## Learning Objectives

After completing this chapter, you will be able to:
- Objective 1
- Objective 2
- Objective 3

## Introduction

Your chapter introduction here...

## Main Content

### Section Title

Your content here...

## Practical Example

```bash
# Code example
echo "Hello, Robotics!"
```

## Summary

Key takeaways from this chapter...

## Exercises

1. Exercise 1 description
2. Exercise 2 description
```

### 2. Add Code Blocks

Use these syntaxes for different code types:

```javascript
// JavaScript/Node.js code
const robot = new Robot();
```

```bash
# Shell commands
npm install robot-package
```

```python
# Python code
import robotics
robot.move_forward()
```

## Building for Production

### 1. Build Static Files

```bash
npm run build
```

### 2. Serve Locally to Test

```bash
npm run serve
```

## Deployment

### GitHub Pages Deployment

1. Configure your `docusaurus.config.js` with your GitHub repository details
2. Run the deployment command:

```bash
npm run deploy
```

## Troubleshooting

### Common Issues

**Issue**: Module not appearing in sidebar
**Solution**: Verify the file paths in `sidebars.js` match your actual file locations

**Issue**: Images not loading
**Solution**: Place images in `static/img/` and reference with `/img/filename.jpg`

**Issue**: Build fails with syntax errors
**Solution**: Check Markdown syntax, especially frontmatter and code block formatting

### Getting Help

- Check the Docusaurus documentation: https://docusaurus.io/
- Review the official Markdown syntax: https://daringfireball.net/projects/markdown/
- Verify all code examples in the context of robotics applications