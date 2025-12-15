import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './ProgressTracker.module.css';

interface ProgressData {
  chapterId: string;
  completionPercentage: number;
  timeSpent: number; // in seconds
  lastAccessed?: string;
}

interface ProgressTrackerProps {
  className?: string;
  initialProgress?: ProgressData[];
}

const ProgressTracker: React.FC<ProgressTrackerProps> = ({ 
  className,
  initialProgress = []
}) => {
  const [progressData, setProgressData] = useState<ProgressData[]>([]);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // In a real implementation, this would fetch from the API
    // For now, we'll use the initial data or mock data
    const loadData = () => {
      if (initialProgress.length > 0) {
        setProgressData(initialProgress);
      } else {
        // Mock data for demonstration
        setProgressData([
          { chapterId: 'module-1/week-1-introduction', completionPercentage: 100, timeSpent: 1800 },
          { chapterId: 'module-1/week-2-nodes-topics', completionPercentage: 100, timeSpent: 2400 },
          { chapterId: 'module-1/week-3-services-actions', completionPercentage: 75, timeSpent: 1200 },
          { chapterId: 'module-1/week-4-tf-urdf', completionPercentage: 0, timeSpent: 0 },
        ]);
      }
      setIsLoading(false);
    };

    loadData();
  }, [initialProgress]);

  const totalChapters = progressData.length;
  const completedChapters = progressData.filter(p => p.completionPercentage === 100).length;
  const overallProgress = totalChapters > 0 
    ? Math.round((completedChapters / totalChapters) * 100) 
    : 0;

  if (isLoading) {
    return (
      <div className={clsx(styles.progressTracker, className)}>
        <div className={styles.loading}>Loading progress...</div>
      </div>
    );
  }

  return (
    <div className={clsx(styles.progressTracker, className)}>
      <div className={styles.header}>
        <h3>Learning Progress</h3>
        <div className={styles.overallProgress}>
          <span className={styles.percentage}>{overallProgress}%</span>
          <span className={styles.statusText}>
            {completedChapters} of {totalChapters} chapters completed
          </span>
        </div>
      </div>

      <div className={styles.progressBar}>
        <div 
          className={styles.progressFill}
          style={{ width: `${overallProgress}%` }}
        ></div>
      </div>

      <div className={styles.chapterList}>
        {progressData.map((progress) => (
          <div key={progress.chapterId} className={styles.chapterItem}>
            <div className={styles.chapterInfo}>
              <span className={styles.chapterId}>
                {progress.chapterId.split('/').pop()}
              </span>
              <span className={styles.timeSpent}>
                {Math.round(progress.timeSpent / 60)} min
              </span>
            </div>
            <div className={styles.chapterProgress}>
              <div 
                className={clsx(
                  styles.chapterProgressFill,
                  progress.completionPercentage === 100 && styles.completed
                )}
                style={{ width: `${progress.completionPercentage}%` }}
              ></div>
              <span className={styles.chapterPercentage}>
                {progress.completionPercentage}%
              </span>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default ProgressTracker;