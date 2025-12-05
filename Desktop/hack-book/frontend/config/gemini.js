// frontend/config/gemini.js

/**
 * Configuration for the Google Gemini API.
 *
 * It is recommended to use environment variables for the API key.
 */
const geminiConfig = {
  apiKey: process.env.REACT_APP_GEMINI_API_KEY,
  // Other options can be added here, for example:
  // generationConfig: {
  //   temperature: 0.9,
  //   topK: 1,
  //   topP: 1,
  //   maxOutputTokens: 2048,
  // },
};

export default geminiConfig;
