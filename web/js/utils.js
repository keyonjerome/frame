/**
 * Small utility helpers for the Video Library frontend.
 * No external dependencies; kept testable in Node.
 */

/**
 * Build the download href for a given filename using the Flask route pattern.
 * We only URL-encode the path segment (filename).
 * @param {string} filename
 * @returns {string}
 */
function buildDownloadHref(filename) {
  if (typeof filename !== 'string' || filename.length === 0) return '/download/';
  return `/download/${encodeURIComponent(filename)}`;
}

/**
 * Construct an absolute URL from a relative/path URL and an origin.
 * Fallbacks to string concatenation when URL API is unavailable.
 * @param {string} href relative or absolute href
 * @param {string} origin like "http://localhost:5000" (no trailing slash)
 * @returns {string}
 */
function toAbsoluteUrl(href, origin) {
  if (typeof href !== 'string') href = '';
  if (typeof origin !== 'string' || origin.length === 0) return href;
  try {
    return new URL(href, origin).toString();
  } catch {
    // Basic fallback
    const left = origin.endsWith('/') ? origin.slice(0, -1) : origin;
    const right = href.startsWith('/') ? href : `/${href}`;
    return `${left}${right}`;
  }
}

module.exports = { buildDownloadHref, toAbsoluteUrl };