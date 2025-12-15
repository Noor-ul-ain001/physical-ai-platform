import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import '@testing-library/jest-dom';
import BookmarkButton from '../components/buttons/BookmarkButton';

// Mock localStorage
const localStorageMock = {
  getItem: jest.fn(),
  setItem: jest.fn(),
  removeItem: jest.fn(),
  clear: jest.fn(),
};
global.localStorage = localStorageMock;

describe('BookmarkButton', () => {
  const defaultProps = {
    chapterId: 'module-1/week-1-introduction',
  };

  beforeEach(() => {
    jest.clearAllMocks();
    localStorageMock.getItem.mockReturnValue('[]');
  });

  it('renders with correct initial state', () => {
    render(<BookmarkButton {...defaultProps} />);
    expect(screen.getByText('Bookmark')).toBeInTheDocument();
    expect(screen.getByLabelText('Add bookmark')).toBeInTheDocument();
  });

  it('changes state when clicked', () => {
    render(<BookmarkButton {...defaultProps} />);
    
    const button = screen.getByRole('button');
    fireEvent.click(button);
    
    // The button text should change to "Bookmarked" after clicking
    expect(screen.getByText('Bookmarked')).toBeInTheDocument();
  });

  it('has correct aria-label when bookmarked', () => {
    localStorageMock.getItem.mockReturnValue(
      JSON.stringify([{ id: 'module-1/week-1-introduction', chapterId: 'module-1/week-1-introduction' }])
    );
    
    render(<BookmarkButton {...defaultProps} />);
    
    expect(screen.getByLabelText('Remove bookmark')).toBeInTheDocument();
  });
});