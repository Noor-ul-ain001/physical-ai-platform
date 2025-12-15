# Deployment Information

This project is configured to be deployed on Vercel for the frontend. The following information is relevant for deployment:

## Build Configuration

The project uses Docusaurus 3.0 which generates static files. The build command is:

```
npm run build
```

## Vercel Deployment Settings

- Build Command: `npm run build`
- Output Directory: `build`
- Install Command: `npm install`

## Environment Variables

For production, you'll need to set the following environment variables in your Vercel dashboard:

```
NEXT_PUBLIC_API_URL=https://your-backend-api.com
NEXT_PUBLIC_BETTER_AUTH_URL=https://your-auth-domain.com
```

## Performance Considerations

This platform has been optimized for fast loading with:
- Code splitting for React components
- Optimized images with WebP format
- Minimal external dependencies
- Prefetching of frequently accessed pages