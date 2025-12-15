import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './dashboard.module.css';

type ModuleProgress = {
  id: string;
  title: string;
  weeks: number;
  completed: number;
  progress: number; // 0-100
};

export default function DashboardPage(): JSX.Element {
  const modules: ModuleProgress[] = [
    {
      id: 'module-1',
      title: 'ROS 2 Fundamentals',
      weeks: 4,
      completed: 2,
      progress: 50
    },
    {
      id: 'module-2',
      title: 'Simulation',
      weeks: 3,
      completed: 0,
      progress: 0
    },
    {
      id: 'module-3',
      title: 'NVIDIA Isaac Sim',
      weeks: 3,
      completed: 0,
      progress: 0
    },
    {
      id: 'module-4',
      title: 'VLA Models',
      weeks: 3,
      completed: 0,
      progress: 0
    }
  ];

  const completedChapters = 8;
  const totalChapters = 13;
  const overallProgress = Math.round((completedChapters / totalChapters) * 100);

  return (
    <Layout title="Dashboard" description="Your Physical AI & Humanoid Robotics learning dashboard">
      <main className={styles.main}>
        <div className={styles.container}>
          <header className={styles.header}>
            <h1>Learning Dashboard</h1>
            <p>Welcome back! Here's your progress in the Physical AI & Humanoid Robotics curriculum.</p>
          </header>

          <section className={styles.overview}>
            <div className={styles.card}>
              <h2>Overall Progress</h2>
              <div className={styles.progressContainer}>
                <div className={styles.progressBar}>
                  <div 
                    className={styles.progressFill} 
                    style={{ width: `${overallProgress}%` }}
                  ></div>
                </div>
                <span className={styles.progressText}>{overallProgress}% Complete</span>
              </div>
              <p>{completedChapters} of {totalChapters} chapters completed</p>
            </div>

            <div className={styles.card}>
              <h2>Quick Actions</h2>
              <div className={styles.quickActions}>
                <Link to="/docs/intro" className={styles.actionButton}>
                  Continue Learning
                </Link>
                <Link to="/docs/module-1/week-3-services-actions" className={styles.actionButton}>
                  Resume Module 1
                </Link>
                <Link to="/chat" className={styles.actionButton}>
                  Ask Assistant
                </Link>
              </div>
            </div>
          </section>

          <section className={styles.modules}>
            <h2>Curriculum Progress</h2>
            <div className={styles.moduleGrid}>
              {modules.map(module => (
                <div key={module.id} className={styles.moduleCard}>
                  <div className={styles.moduleHeader}>
                    <h3>{module.title}</h3>
                    <span className={styles.progressText}>{module.progress}%</span>
                  </div>
                  <div className={styles.moduleProgress}>
                    <div 
                      className={styles.progressFill} 
                      style={{ width: `${module.progress}%` }}
                    ></div>
                  </div>
                  <div className={styles.moduleDetails}>
                    <span>{module.completed} of {module.weeks} weeks completed</span>
                    <Link to={`/docs/${module.id}/week-1-introduction`}>
                      Continue
                    </Link>
                  </div>
                </div>
              ))}
            </div>
          </section>

          <section className={styles.recentActivity}>
            <h2>Recent Activity</h2>
            <div className={styles.activityList}>
              <div className={styles.activityItem}>
                <span className={styles.activityIcon}>📖</span>
                <div className={styles.activityContent}>
                  <h4>Completed: Services and Actions</h4>
                  <p>Finished Week 3 of Module 1</p>
                  <span className={styles.activityTime}>2 hours ago</span>
                </div>
              </div>
              <div className={styles.activityItem}>
                <span className={styles.activityIcon}>❓</span>
                <div className={styles.activityContent}>
                  <h4>Asked: Difference between services and actions?</h4>
                  <p>Got help from the Physical AI Assistant</p>
                  <span className={styles.activityTime}>1 day ago</span>
                </div>
              </div>
              <div className={styles.activityItem}>
                <span className={styles.activityIcon}>🎯</span>
                <div className={styles.activityContent}>
                  <h4>Completed Quiz: ROS 2 Basics</h4>
                  <p>Scored 90% on the ROS 2 introduction quiz</p>
                  <span className={styles.activityTime}>2 days ago</span>
                </div>
              </div>
            </div>
          </section>
        </div>
      </main>
    </Layout>
  );
}