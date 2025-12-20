# Image Optimization Guidelines for Module 3

## Recommended Formats

- **Photographs and complex images**: Use JPEG with 80-90% quality
- **Graphics with limited colors**: Use PNG for transparency or sharp edges
- **Simple diagrams and icons**: Use SVG for scalability
- **Animations**: Use GIF or MP4 for video content

## Size Guidelines

- **Photographs**: Max 1920px width at 72 DPI
- **Diagrams**: Max 1200px width at 72 DPI
- **Icons**: Use vector formats (SVG) when possible
- **Screenshots**: Crop to relevant area, max 1500px width

## Naming Convention

Use descriptive, lowercase names with hyphens:
- Good: `humanoid-robot-simulation.jpg`
- Bad: `IMG_1234.JPG` or `robot pic.png`

## Accessibility

- Include descriptive alt text for all images
- Provide captions for complex diagrams
- Ensure sufficient contrast for text overlays
- Consider colorblind-friendly palettes

## Performance

- Compress images before uploading
- Use WebP format for modern browsers (with fallbacks)
- Implement lazy loading for images below the fold
- Consider using image CDNs for better delivery

## Tools for Optimization

- **Online**: TinyPNG, ImageOptim, Squoosh
- **Desktop**: GIMP, Photoshop with save-for-web
- **CLI**: ImageMagick, optipng, jpegoptim
- **Build tools**: Webpack image loaders, Gatsby image processing