# UI Component Design System

This document outlines the design system for UI components in the Physical AI & Humanoid Robotics educational platform.

## Color Palette

### Primary Colors
- **Robotic Blue**: `#2563eb` - Used for primary actions, links, and highlights
- **Circuit Green**: `#16a34a` - Used for success states, confirmations, and positive actions
- **Hardware Orange**: `#ea580c` - Used for warnings and hardware-related elements
- **Sensor Purple**: `#9333ea` - Used for sensor-related information and special highlights

### Neutral Colors
- **Text Primary**: `#1f2937` - Main text color
- **Text Secondary**: `#6b7280` - Secondary text, captions
- **Background**: `#f9fafb` - Page and component backgrounds
- **Border**: `#e5e7eb` - Component borders and dividers

## Typography

### Font Stack
```css
font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, 'Open Sans', 'Helvetica Neue', sans-serif;
```

### Scale
- **Heading 1**: 2.5rem (40px), font-weight: 700
- **Heading 2**: 2rem (32px), font-weight: 700
- **Heading 3**: 1.5rem (24px), font-weight: 600
- **Heading 4**: 1.25rem (20px), font-weight: 600
- **Body Large**: 1.125rem (18px), font-weight: 400
- **Body Regular**: 1rem (16px), font-weight: 400
- **Body Small**: 0.875rem (14px), font-weight: 400
- **Caption**: 0.75rem (12px), font-weight: 400

## Spacing System
Based on a 4px base unit:
- `space-1`: 4px
- `space-2`: 8px
- `space-3`: 12px
- `space-4`: 16px
- `space-5`: 20px
- `space-6`: 24px
- `space-8`: 32px
- `space-10`: 40px
- `space-12`: 48px

## Component Specifications

### 1. Buttons

#### Styles
- **Primary**: Background robotic-blue, white text, full rounded corners
- **Secondary**: White background, robotic-blue border, robotic-blue text
- **Ghost**: Transparent background, robotic-blue text
- **Destructive**: Red background, white text

#### Sizes
- **Small**: 2rem height, 0.5rem horizontal padding, 0.75rem font
- **Medium**: 2.5rem height, 1rem horizontal padding, 1rem font
- **Large**: 3rem height, 1.5rem horizontal padding, 1.125rem font

#### States
- **Default**: Normal appearance
- **Hover**: Slight opacity change or color shift
- **Active**: Pressed appearance
- **Focus**: Visible focus ring (WCAG compliant)
- **Disabled**: Reduced opacity, not interactive

### 2. Cards
- **Background**: White with subtle shadow: `0 4px 6px -1px rgba(0, 0, 0, 0.1)`
- **Border Radius**: 8px
- **Padding**: 1.5rem
- **Content Spacing**: 1rem between sections

### 3. Forms
- **Input Fields**: 
  - Height: 3rem
  - Border: 1px solid border color
  - Border radius: 4px
  - Padding: 0 1rem
  - Focus: Border robotic-blue, shadow effect
- **Labels**: Body regular font, secondary text color
- **Error State**: Red border, error text below

### 4. Interactive Components
For specialized components like InteractiveCodeBlock and HardwareTable:
- Maintain consistent padding and spacing
- Use platform colors appropriately
- Ensure full responsiveness
- Include proper accessibility attributes

## Iconography
- Use consistent 24x24px icons for interface elements
- 16x16px for small icons in text
- Prefer SVG format for crisp rendering
- Use appropriate semantic colors for different contexts

## Responsive Breakpoints
- **Mobile**: Up to 640px
- **Tablet**: 641px to 1024px
- **Desktop**: 1025px and above

### Responsive Guidelines
- Stack elements vertically on mobile
- Maintain readable font sizes (min 16px)
- Ensure touch targets are at least 44px
- Preserve important content hierarchy

## Accessibility Standards (WCAG 2.1 AA)
- All interactive elements must have proper focus states
- Color contrast ratios must be at least 4.5:1 for normal text
- Provide alternative text for all meaningful images
- Use semantic HTML elements appropriately
- Ensure keyboard navigability

## Animation Guidelines
- Use reduced motion for users who prefer it
- Animations should last 200-300ms
- Use `ease-in-out` timing function
- Only animate for functional or feedback purposes
- Avoid flashing content (less than 3 times per second)

## Implementation Notes
- All components should be built with TypeScript
- Use CSS Modules or Tailwind for styling
- Include proper JSDoc documentation
- Export components with barrel files
- Provide Storybook stories where applicable