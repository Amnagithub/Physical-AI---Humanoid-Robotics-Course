# Quickstart Guide: Vision-Language-Action (VLA) Module

## Overview
This guide will help you set up and run Module 4 of the Physical AI & Humanoid Robotics course, focusing on Vision-Language-Action systems for humanoid robots.

## Prerequisites
Before starting with this module, ensure you have:

1. **Development Environment**:
   - Node.js 18+ (for Docusaurus documentation)
   - Python 3.8+
   - ROS 2 Humble Hawksbill installed and sourced

2. **API Access**:
   - OpenAI API key for Whisper and LLM functionality
   - (Optional) Access to NVIDIA Isaac Sim for advanced simulation

3. **Course Prerequisites**:
   - Basic understanding of ROS 2 concepts
   - Familiarity with Python programming
   - Completion of Modules 1-3 (recommended)

## Setup Instructions

### 1. Install Docusaurus Dependencies
```bash
cd frontend
npm install
```

### 2. Verify Current Setup
```bash
npm run start
```
The development server should start and be accessible at `http://localhost:3000/Physical-AI---Humanoid-Robotics-Course/`

### 3. Module Structure
The VLA module will be organized as follows:
```
docs/modules/module-4/
├── index.md                    # Module overview
├── chapter-1-voice-control.md  # Voice-to-Action systems
├── chapter-2-llm-planning.md   # Cognitive planning with LLMs
├── chapter-3-autonomous-capstone.md # Capstone project
├── glossary.md                 # VLA-specific terminology
├── troubleshooting.md          # Common issues and solutions
└── examples/                   # Code examples and resources
```

## Chapter 1: Voice Control Setup

### Key Components
- Speech recognition using OpenAI Whisper
- Voice command processing pipeline
- Integration with ROS 2 action servers

### Example Setup
```bash
# Navigate to examples directory
cd docs/modules/module-4/examples

# Install Python dependencies
pip install openai speech-recognition rospy

# Run voice command example
python voice_command_demo.py
```

## Chapter 2: LLM-Based Planning Setup

### Key Components
- Natural language goal parsing
- LLM-based task decomposition
- ROS 2 action sequence generation

### Example Setup
```bash
# Install LLM dependencies
pip install openai langchain

# Run planning example
python llm_planning_demo.py
```

## Chapter 3: Autonomous Capstone Setup

### Key Components
- Integration of voice control and LLM planning
- End-to-end autonomous behavior
- Simulation-based testing

### Example Setup
```bash
# Run complete VLA system
python vla_system_demo.py
```

## Running the Module

1. **Start the Docusaurus Development Server**:
```bash
cd frontend
npm run start
```

2. **Access the Module**:
   - Open your browser to: `http://localhost:3000/Physical-AI---Humanoid-Robotics-Course/`
   - Navigate to: `Modules > Module 4: Vision-Language-Action (VLA)`

3. **Follow Along with Examples**:
   - Each chapter includes practical examples
   - Code examples are located in the `examples/` directory
   - Follow the step-by-step instructions for hands-on learning

## Troubleshooting

### Common Issues

**Issue**: Docusaurus server won't start
- **Solution**: Check that you're in the `frontend` directory and run `npm install` to ensure all dependencies are installed

**Issue**: OpenAI API errors
- **Solution**: Verify your API key is set in environment variables:
```bash
export OPENAI_API_KEY="your-api-key-here"
```

**Issue**: ROS 2 connection problems
- **Solution**: Ensure ROS 2 environment is properly sourced:
```bash
source /opt/ros/humble/setup.bash
```

## Next Steps

After completing this quickstart:
1. Proceed through each chapter in sequence
2. Complete the hands-on examples
3. Work on the capstone project in Chapter 3
4. Review the glossary for key terminology
5. Use the troubleshooting guide if you encounter issues