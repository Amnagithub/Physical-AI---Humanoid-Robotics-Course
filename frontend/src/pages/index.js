import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import FloatingChatButton from '../components/FloatingChatButton';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/modules/ros2-nervous-system/intro">
            Explore Course Modules
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Course - Understanding the Robotic Nervous System with ROS 2">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h3>Comprehensive Curriculum</h3>
                <p>
                  Learn about humanoid robotics, ROS 2, and the principles of Physical AI through structured modules.
                </p>
              </div>
              <div className="col col--4">
                <h3>Hands-on Learning</h3>
                  <p>
                    Practical exercises and real-world examples to help you understand robotic nervous systems.
                  </p>
              </div>
              <div className="col col--4">
                <h3>Expert Guidance</h3>
                <p>
                  Developed by experts in humanoid robotics and AI to provide the best learning experience.
                </p>
              </div>
            </div>
          </div>
        </section>

      </main>
      <FloatingChatButton apiBaseUrl="http://localhost:8001" />
    </Layout>
  );
}