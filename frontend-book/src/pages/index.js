import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">Physical AI & Humanoid Robotics</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/module-1/intro">
            Start Reading
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
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics textbook">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--12">
                <p>
                  Step into the world where AI leaves the screen and enters reality. This book introduces Physical AI—intelligent systems that perceive, reason, and act in the real world. You'll learn how digital intelligence connects to a physical body through humanoid robots, explore simulation techniques, and understand how perception, language, and action combine for natural human-robot interaction.
                </p>
                <p>
                  By the end, you'll be able to design, simulate, and control humanoid robots using ROS 2, Gazebo, and NVIDIA Isaac, bringing AI from software into autonomous, embodied agents.
                </p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}