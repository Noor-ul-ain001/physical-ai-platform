import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './DiagramViewer.module.css';

type DiagramViewerProps = {
  src: string;
  alt: string;
  caption?: string;
  width?: string;
  height?: string;
};

export default function DiagramViewer({ 
  src, 
  alt, 
  caption,
  width = '100%',
  height = 'auto'
}: DiagramViewerProps): JSX.Element {
  const [isLoading, setIsLoading] = useState(true);
  const [hasError, setHasError] = useState(false);

  const handleLoad = () => {
    setIsLoading(false);
  };

  const handleError = () => {
    setIsLoading(false);
    setHasError(true);
  };

  return (
    <div className={clsx(styles.diagramContainer)}>
      <div className={clsx(styles.diagramWrapper)} style={{ width, height }}>
        {isLoading && <div className={clsx(styles.loadingIndicator)}>Loading diagram...</div>}
        {hasError && (
          <div className={clsx(styles.errorIndicator)}>
            Failed to load diagram: {alt}
          </div>
        )}
        <img
          src={src}
          alt={alt}
          className={clsx(
            styles.diagramImage,
            isLoading && styles.hidden,
            hasError && styles.hidden
          )}
          onLoad={handleLoad}
          onError={handleError}
        />
      </div>
      {caption && <div className={clsx(styles.caption)}>{caption}</div>}
    </div>
  );
}