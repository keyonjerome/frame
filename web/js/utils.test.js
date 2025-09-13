const { buildDownloadHref, toAbsoluteUrl } = require('./utils');

describe('buildDownloadHref', () => {
  test('encodes spaces and unicode', () => {
    expect(buildDownloadHref('my video 01.mp4')).toBe('/download/my%20video%2001.mp4');
    expect(buildDownloadHref('café.mov')).toBe('/download/caf%C3%A9.mov');
  });

  test('handles simple names', () => {
    expect(buildDownloadHref('file.mkv')).toBe('/download/file.mkv');
  });

  test('handles empty/invalid', () => {
    expect(buildDownloadHref('')).toBe('/download/');
    expect(buildDownloadHref(null)).toBe('/download/');
    expect(buildDownloadHref(undefined)).toBe('/download/');
  });
});

describe('toAbsoluteUrl', () => {
  test('combines with origin', () => {
    const origin = 'http://localhost:5000';
    expect(toAbsoluteUrl('/download/file.mp4', origin)).toBe('http://localhost:5000/download/file.mp4');
  });

  test('works with full URLs', () => {
    const full = 'http://example.com/a/b';
    expect(toAbsoluteUrl(full, 'http://localhost:5000')).toBe(full);
  });

  test('fallback join when URL API fails', () => {
    // Simulate bad origin to trigger fallback branch
    expect(toAbsoluteUrl('/path', 'not-a-valid-origin')).toBe('not-a-valid-origin/path');
  });
});
