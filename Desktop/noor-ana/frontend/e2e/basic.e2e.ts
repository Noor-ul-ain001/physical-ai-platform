import { test, expect } from '@playwright/test';

test.describe('Physical AI Platform E2E Tests', () => {
  test.beforeEach(async ({ page }) => {
    // Navigate to the platform before each test
    await page.goto('http://localhost:3000'); // Assuming development server
  });

  test('should display the homepage with curriculum modules', async ({ page }) => {
    // Check if the main heading is visible
    await expect(page.locator('h1')).toContainText('Physical AI & Humanoid Robotics');

    // Check if navigation sidebar with modules is present
    await expect(page.locator('nav')).toBeVisible();
    
    // Check if at least one module is visible in the sidebar
    await expect(page.locator('text=Module 1: ROS 2 Fundamentals')).toBeVisible();
    await expect(page.locator('text=Module 2: Simulation')).toBeVisible();
  });

  test('should allow navigation to a curriculum module', async ({ page }) => {
    // Click on a module in the sidebar
    await page.click('text=Module 1: ROS 2 Fundamentals');
    
    // Wait for navigation and check for content
    await page.waitForURL('**/module-1**');
    
    // Verify we're on the module page
    await expect(page.locator('h1')).toContainText('Introduction to ROS 2');
  });

  test('should show the chatbot interface', async ({ page }) => {
    // Find and interact with the chatbot toggle
    const chatbotToggle = page.locator('button:has-text("🤖")');
    await expect(chatbotToggle).toBeVisible();
    
    // Click the chatbot toggle
    await chatbotToggle.click();
    
    // Verify the chatbot widget appears
    await expect(page.locator('text=Physical AI Assistant')).toBeVisible();
  });

  test('should maintain user preferences', async ({ page }) => {
    // Visit the site and simulate user interaction
    await page.goto('http://localhost:3000/docs/intro');
    
    // Check if personalization button is present
    const personalizeBtn = page.locator('text=Personalize');
    await expect(personalizeBtn).toBeVisible();
    
    // Note: We can't fully test personalization without auth setup
    // but we can check if the button exists
  });
});