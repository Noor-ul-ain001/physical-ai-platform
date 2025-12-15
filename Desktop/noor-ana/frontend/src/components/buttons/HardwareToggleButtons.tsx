import React, { useState } from 'react';
import clsx from 'clsx';
import Button from '../common/Button';
import styles from './HardwareToggleButtons.module.css';

interface HardwareToggleButtonsProps {
  className?: string;
  onHardwareChange?: (hardware: 'cloud' | 'jetson' | 'robot') => void;
}

const HardwareToggleButtons: React.FC<HardwareToggleButtonsProps> = ({ 
  className,
  onHardwareChange
}) => {
  const [selectedHardware, setSelectedHardware] = useState<'cloud' | 'jetson' | 'robot'>('cloud');

  const handleSelect = (hardware: 'cloud' | 'jetson' | 'robot') => {
    setSelectedHardware(hardware);
    onHardwareChange?.(hardware);
  };

  return (
    <div className={clsx(styles.toggleContainer, className)}>
      <Button
        variant={selectedHardware === 'cloud' ? 'primary' : 'secondary'}
        size="sm"
        onClick={() => handleSelect('cloud')}
      >
        Cloud
      </Button>
      <Button
        variant={selectedHardware === 'jetson' ? 'primary' : 'secondary'}
        size="sm"
        onClick={() => handleSelect('jetson')}
      >
        Jetson
      </Button>
      <Button
        variant={selectedHardware === 'robot' ? 'primary' : 'secondary'}
        size="sm"
        onClick={() => handleSelect('robot')}
      >
        Robot
      </Button>
    </div>
  );
};

export default HardwareToggleButtons;