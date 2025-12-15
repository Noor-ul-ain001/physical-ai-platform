# Contributing to Physical AI & Humanoid Robotics Educational Platform

We're excited that you're interested in contributing to the Physical AI & Humanoid Robotics Educational Platform! This document outlines the process for contributing to our project.

## Table of Contents
- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Workflow](#development-workflow)
- [Technical Guidelines](#technical-guidelines)
- [Testing](#testing)
- [Pull Request Process](#pull-request-process)
- [Community](#community)

## Code of Conduct

This project and everyone participating in it is governed by our Code of Conduct. By participating, you are expected to uphold this code. Please report unacceptable behavior to the project maintainers.

## Getting Started

### Fork the Repository
1. Fork the repository on GitHub
2. Clone your fork locally: `git clone https://github.com/YOUR-USERNAME/noor-ana.git`
3. Add the upstream repository: `git remote add upstream https://github.com/ORIGINAL-OWNER/noor-ana.git`

### Set Up Your Development Environment
Follow the setup instructions in the [README](README.md) to install dependencies and configure your local environment.

### Find Something to Work On
- Look through the issues to find something that interests you
- If you're new, look for issues labeled "good first issue"
- Comment on the issue to let others know you're working on it

## Development Workflow

### Create a Branch
```bash
git checkout -b feature/your-feature-name
```

### Make Your Changes
- Write clean, well-documented code
- Follow the coding standards of the project
- Write tests for your code when applicable
- Test your changes thoroughly

### Commit Your Changes
Write clear, descriptive commit messages:

```
feat: add new component for hardware specifications

- Create HardwareTable component
- Add responsive design
- Implement accessibility features
- Write unit tests
```

### Update Your Branch
Keep your branch up to date with the upstream main branch:

```bash
git fetch upstream
git rebase upstream/main
```

## Technical Guidelines

### Frontend
- Write TypeScript for type safety
- Use functional components with hooks
- Follow the existing component structure
- Style with Tailwind CSS and the provided design system
- Ensure responsive design for all screen sizes
- Implement accessibility best practices

### Backend
- Use FastAPI for API endpoints
- Implement proper error handling
- Validate request/response data with Pydantic
- Write async code where appropriate
- Follow security best practices
- Use proper logging

### Documentation
- Update documentation when adding new features
- Write clear, concise comments
- Include examples where appropriate
- Ensure documentation is accessible and easy to understand

## Testing

### Frontend Tests
- Write unit tests for components using React Testing Library
- Add integration tests for complex features
- Run tests with: `npm test`

### Backend Tests
- Write unit tests for services and API endpoints
- Include integration tests for database operations
- Run tests with: `pytest`

### End-to-End Tests
- Add Playwright tests for critical user flows
- Run E2E tests with: `npm run test:e2e`

## Pull Request Process

1. Ensure your code follows the project's style guidelines
2. Write clear, descriptive commit messages
3. Include tests for new functionality
4. Update documentation as needed
5. Submit your pull request with a clear description of what you've done
6. Link any issues that your pull request addresses
7. Be responsive to feedback during the review process

### Pull Request Guidelines
- Keep pull requests focused on a single issue/feature
- Write a clear, descriptive title
- Include a detailed description of what the PR does
- List any breaking changes
- Update the README if necessary

### Review Process
- PRs require approval from at least one maintainer
- Address all review comments before merging
- PRs should be merged by the reviewer or by the submitter after approval

## Community

### Questions or Problems?
- Use GitHub issues for bug reports and feature requests
- For general questions, open a discussion
- Join our community if we have one (specify how)

### Reporting Issues
- Use the appropriate issue template
- Provide as much detail as possible
- Include steps to reproduce the issue
- Mention your environment (OS, browser, version)

## Style Guides

### Git Commit Messages
- Use the present tense ("Add feature" not "Added feature")
- Use the imperative mood ("Move cursor to..." not "Moves cursor to...")
- Limit the first line to 72 characters or less
- Reference issues and pull requests after the description

### Code Style
- Follow the existing code style of the project
- Use meaningful variable and function names
- Keep functions small and focused
- Comment complex logic, but prioritize readable code

Thank you for contributing to the Physical AI & Humanoid Robotics Educational Platform!