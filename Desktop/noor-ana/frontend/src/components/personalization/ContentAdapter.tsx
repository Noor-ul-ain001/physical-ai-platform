import React, { useEffect, useState } from 'react';
import clsx from 'clsx';
import styles from './ContentAdapter.module.css';

type ContentAdapterProps = {
  children: React.ReactNode;
  showSections: string[];
  hideSections: string[];
  codeComplexity: 'simple' | 'standard' | 'advanced';
  hardwareInstructions: 'cloud' | 'jetson' | 'full_robot';
  className?: string;
};

export default function ContentAdapter({ 
  children, 
  showSections, 
  hideSections, 
  codeComplexity,
  hardwareInstructions,
  className 
}: ContentAdapterProps): JSX.Element {
  const [processedContent, setProcessedContent] = useState<React.ReactNode>(children);

  useEffect(() => {
    // Process the content based on personalization rules
    setProcessedContent(processContent());
  }, [showSections, hideSections, codeComplexity, hardwareInstructions]);

  const processContent = (): React.ReactNode => {
    // This is a simplified implementation - in a real system, 
    // this would recursively process the content tree
    if (typeof children === 'string') {
      // If it's a string, return as is
      return children;
    }
    
    // If it's a React element, process its props and children
    if (React.isValidElement(children)) {
      // For code blocks, adjust complexity if needed
      if (children.props.className?.includes('code-block')) {
        return React.cloneElement(children, {
          ...children.props,
          children: adjustCodeComplexity(children.props.children)
        });
      }
      
      // For other elements, check if they should be shown/hidden
      const sectionId = children.props['data-section-id'];
      if (sectionId) {
        if (hideSections.includes(sectionId)) {
          return null; // Hide this section
        }
        if (showSections.includes(sectionId)) {
          // Show this section with potential modifications
          return React.cloneElement(children, {
            ...children.props,
            className: clsx(children.props.className, styles.personalizedSection)
          });
        }
      }
      
      // Recursively process children
      const processedChildren = React.Children.map(children.props.children, (child) => {
        if (React.isValidElement(child)) {
          const childSectionId = child.props['data-section-id'];
          if (childSectionId) {
            if (hideSections.includes(childSectionId)) {
              return null;
            }
            if (showSections.includes(childSectionId)) {
              return React.cloneElement(child, {
                ...child.props,
                className: clsx(child.props.className, styles.personalizedSection)
              });
            }
          }
        }
        return child;
      });
      
      return React.cloneElement(children, {
        ...children.props,
        children: processedChildren
      });
    }
    
    // If it's not a string or React element, return as is
    return children;
  };

  const adjustCodeComplexity = (code: React.ReactNode): React.ReactNode => {
    if (typeof code === 'string') {
      // Apply complexity transformation to code strings
      let modifiedCode = code;
      
      if (codeComplexity === 'simple') {
        // Simplify complex code examples
        modifiedCode = code
          .replace(/# Advanced implementation/g, '# Simple implementation')
          .replace(/complex_function\(\)/g, 'simple_function()')
          .replace(/advanced_pattern = /g, 'simple_pattern = ');
      } else if (codeComplexity === 'advanced') {
        // Enhance with advanced concepts
        modifiedCode = code
          .replace(/# Basic implementation/g, '# Advanced implementation')
          .replace(/basic_function\(\)/g, 'advanced_function()')
          .replace(/simple_pattern = /g, 'advanced_pattern = ');
      }
      
      return modifiedCode;
    }
    
    // If code is not a string, return as is
    return code;
  };

  const getHardwareInstructions = (): string => {
    switch (hardwareInstructions) {
      case 'cloud':
        return 'Follow the cloud simulation instructions';
      case 'jetson':
        return 'Follow the Jetson Orin setup instructions';
      case 'full_robot':
        return 'Follow the full robot platform instructions';
      default:
        return 'Follow the standard setup instructions';
    }
  };

  // Apply hardware instruction modifications to content
  const applyHardwareInstructions = (content: React.ReactNode): React.ReactNode => {
    if (typeof content === 'string') {
      // Replace placeholder with actual hardware instructions
      return content.replace('{hardware_instructions}', getHardwareInstructions());
    }
    
    if (React.isValidElement(content)) {
      const newProps: any = { ...content.props };
      
      // Process children recursively
      if (content.props.children) {
        newProps.children = React.Children.map(content.props.children, applyHardwareInstructions);
      }
      
      return React.cloneElement(content, newProps);
    }
    
    return content;
  };

  return (
    <div className={clsx(styles.contentAdapter, className)}>
      {applyHardwareInstructions(processedContent)}
    </div>
  );
}