# Data Model: RAG Chatbot

**Feature**: AI-Driven Development Book & RAG Chatbot

This document outlines the key data entities required for the chatbot's functionality. The primary focus is on how content is processed and stored for retrieval.

## Entity: Text Chunk

This is the core entity used for the Retrieval Augmented Generation (RAG) process. The source book content is broken down into these chunks for efficient searching.

| Field          | Type           | Description                                                                 | Example                                     |
|----------------|----------------|-----------------------------------------------------------------------------|---------------------------------------------|
| `id`           | UUID           | The unique identifier for the chunk, generated for storage in Qdrant.       | `6a8e4b2e-0b0b-4b2e-9b0b-3b2e0b0b4b2e`    |
| `content`      | String         | The actual text content of the chunk.                                       | "Spec-driven development is a methodology..." |
| `source`       | String         | The origin of the content, such as the chapter and section name.            | "Chapter 1: Introduction"                   |
| `embedding`    | Vector         | The numerical vector representation of the `content`. Stored in Qdrant.     | `[0.123, 0.456, ...]`                       |

### State Transitions

A `Text Chunk` is immutable. Once created, it is not updated. The set of all chunks changes only when the source book content is updated and the embedding pipeline is re-run.

## Entity: Chat Request (API Model)

This represents an incoming question from the user to the backend API.

| Field          | Type           | Description                                                                 |
|----------------|----------------|-----------------------------------------------------------------------------|
| `query`        | String         | The user's question in natural language.                                    |
| `context`      | String         | (Optional) A specific snippet of text provided by the user.                 |

## Entity: Chat Response (API Model)

This represents the answer sent back to the user from the backend API.

| Field          | Type           | Description                                                                 |
|----------------|----------------|-----------------------------------------------------------------------------|
| `answer`       | String         | The chatbot's generated answer.                                             |
| `sources`      | Array[String]  | A list of source identifiers from the `Text Chunk` entities used to generate the answer. |
