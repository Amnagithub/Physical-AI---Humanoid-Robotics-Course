# Quickstart Guide: Digital Twin Book Module Setup

## Prerequisites

Before starting the setup, ensure you have:

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control
- A GitHub account for hosting (if using GitHub Pages)
- Basic understanding of Docusaurus documentation framework

## Repository Setup

1. **Clone the repository** (if not already done):
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Ensure you're on the correct branch**:
   ```bash
   git checkout 002-digital-twin-book
   ```

## Docusaurus Environment Setup

1. **Install dependencies**:
   ```bash
   npm install
   ```

2. **Start the development server**:
   ```bash
   npm start
   ```
   This will start a local server at http://localhost:3000

## Creating the Module Structure

1. **Create the main module directory**:
   ```bash
   mkdir -p docs/module-2-digital-twin
   ```

2. **Create chapter directories**:
   ```bash
   mkdir -p docs/module-2-digital-twin/chapter-1-gazebo-physics
   mkdir -p docs/module-2-digital-twin/chapter-2-unity-env
   mkdir -p docs/module-2-digital-twin/chapter-3-sensor-sim
   ```

## Setting up Module Index

1. **Create the main module index** at `docs/module-2-digital-twin/index.md`:
   ```markdown
   # The Digital Twin (Gazebo & Unity)

   Welcome to Module 2 of the Physical AI & Humanoid Robotics course. This module focuses on digital twin technologies using Gazebo for physics simulation and Unity for visualization and interaction.

   ## Overview

   In this module, you will learn:
   - How to simulate humanoid robots in Gazebo with realistic physics
   - How to create immersive environments using Unity
   - How to configure and interpret simulated sensor data

   ## Prerequisites

   Before starting this module, you should have:
   - Basic understanding of robotics concepts
   - Familiarity with simulation environments
   - Access to Gazebo and Unity (or cloud-based alternatives)
   ```

## Configuring Navigation

1. **Update the sidebar** in `sidebars.js` to include the new module:
   ```javascript
   module.exports = {
     // ... existing sidebar configuration
     module2: [
       {
         type: 'category',
         label: 'Module 2: The Digital Twin (Gazebo & Unity)',
         items: [
           'module-2-digital-twin/index',
           {
             type: 'category',
             label: 'Chapter 1: Gazebo Physics Simulation',
             items: [
               'module-2-digital-twin/chapter-1-gazebo-physics/index',
               'module-2-digital-twin/chapter-1-gazebo-physics/setup',
               'module-2-digital-twin/chapter-1-gazebo-physics/physics-concepts',
               'module-2-digital-twin/chapter-1-gazebo-physics/humanoid-dynamics',
               'module-2-digital-twin/chapter-1-gazebo-physics/exercises'
             ]
           },
           {
             type: 'category',
             label: 'Chapter 2: Unity Environments and Interaction',
             items: [
               'module-2-digital-twin/chapter-2-unity-env/index',
               'module-2-digital-twin/chapter-2-unity-env/unity-setup',
               'module-2-digital-twin/chapter-2-unity-env/rendering',
               'module-2-digital-twin/chapter-2-unity-env/interaction',
               'module-2-digital-twin/chapter-2-unity-env/exercises'
             ]
           },
           {
             type: 'category',
             label: 'Chapter 3: Humanoid Sensor Simulation',
             items: [
               'module-2-digital-twin/chapter-3-sensor-sim/index',
               'module-2-digital-twin/chapter-3-sensor-sim/sensor-types',
               'module-2-digital-twin/chapter-3-sensor-sim/config-guide',
               'module-2-digital-twin/chapter-3-sensor-sim/data-interpretation',
               'module-2-digital-twin/chapter-3-sensor-sim/exercises'
             ]
           }
         ]
       }
     ]
   };
   ```

## Creating Chapter Content

1. **Create chapter index files** for each chapter with overview content
2. **Add content pages** for each topic within the chapters
3. **Include practical exercises** at the end of each chapter

## Building and Deployment

1. **Build the static site**:
   ```bash
   npm run build
   ```

2. **Serve the build locally** (for testing):
   ```bash
   npm run serve
   ```

3. **Deploy to GitHub Pages** (if applicable):
   ```bash
   GIT_USER=<your-github-username> npm run deploy
   ```

## Verification Steps

1. **Check that all pages render correctly** in the development server
2. **Verify navigation works** between all module pages
3. **Confirm all links are functional** and point to correct locations
4. **Test mobile responsiveness** of the documentation
5. **Validate all code snippets** for accuracy

## Next Steps

1. Add detailed content to each chapter page
2. Include diagrams and images to enhance understanding
3. Create hands-on exercises with expected outcomes
4. Add troubleshooting sections for common issues
5. Review content for technical accuracy