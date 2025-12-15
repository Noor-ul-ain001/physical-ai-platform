import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Button from './Button';
import styles from './BookmarkButton.module.css';
import { BookmarkAPI } from '../../lib/api';
import { useAuth } from '../../contexts/AuthContext';

interface BookmarkButtonProps {
  chapterId: string;
  sectionId?: string;
  className?: string;
}

const BookmarkButton: React.FC<BookmarkButtonProps> = ({
  chapterId,
  sectionId,
  className
}) => {
  const { isAuthenticated } = useAuth();
  const [isBookmarked, setIsBookmarked] = useState(false);
  const [bookmarkId, setBookmarkId] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  // Check if this chapter/section is already bookmarked
  useEffect(() => {
    const checkBookmark = async () => {
      if (!isAuthenticated) {
        setIsBookmarked(false);
        return;
      }

      try {
        // Fetch all bookmarks from backend
        const response = await BookmarkAPI.getAll();
        if (response.data) {
          const bookmark = response.data.find((b: any) =>
            b.chapter_id === chapterId &&
            (sectionId ? b.section_id === sectionId : !b.section_id)
          );

          if (bookmark) {
            setIsBookmarked(true);
            setBookmarkId(bookmark.id);
          } else {
            setIsBookmarked(false);
            setBookmarkId(null);
          }
        }
      } catch (error) {
        console.error('Error checking bookmark status:', error);
      }
    };

    checkBookmark();
  }, [chapterId, sectionId, isAuthenticated]);

  const handleBookmark = async () => {
    if (!isAuthenticated) {
      // Redirect to signup/login if not authenticated
      window.location.href = '/signup';
      return;
    }

    setIsLoading(true);

    try {
      if (isBookmarked && bookmarkId) {
        // Remove bookmark via API
        const response = await BookmarkAPI.delete(bookmarkId);

        if (response.error) {
          throw new Error(response.error);
        }

        setIsBookmarked(false);
        setBookmarkId(null);
      } else {
        // Add bookmark via API
        const response = await BookmarkAPI.create(
          chapterId,
          sectionId,
          `Bookmarked from ${document.title}`
        );

        if (response.error) {
          throw new Error(response.error);
        }

        if (response.data) {
          setIsBookmarked(true);
          setBookmarkId(response.data.id);
        }
      }
    } catch (error) {
      console.error('Error updating bookmark:', error);
      alert('Failed to update bookmark. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Button
      variant={isBookmarked ? 'primary' : 'secondary'}
      size="sm"
      onClick={handleBookmark}
      isLoading={isLoading}
      className={clsx(styles.bookmarkButton, className)}
      aria-label={isBookmarked ? "Remove bookmark" : "Add bookmark"}
    >
      <span className={clsx(styles.icon, isBookmarked && styles.bookmarked)}>
        {isBookmarked ? '★' : '☆'}
      </span>
      <span>{isBookmarked ? 'Bookmarked' : 'Bookmark'}</span>
    </Button>
  );
};

export default BookmarkButton;