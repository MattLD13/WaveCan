const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const { URL } = require('node:url');

const root = path.resolve(__dirname, '..');
const port = Number(process.env.PORT || 4173);
const types = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.cjs': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8', '.png': 'image/png', '.svg': 'image/svg+xml' };

const server = http.createServer((request, response) => {
  const requestPath = decodeURIComponent(new URL(request.url, 'http://127.0.0.1').pathname);
  const relative = requestPath === '/' ? '/index.html' : requestPath;
  const file = path.resolve(root, `.${relative}`);
  if (!file.startsWith(`${root}${path.sep}`)) { response.writeHead(403); response.end('Forbidden'); return; }
  fs.stat(file, (statError, stats) => {
    if (statError || !stats.isFile()) { response.writeHead(404); response.end('Not found'); return; }
    response.writeHead(200, { 'content-type': types[path.extname(file)] || 'application/octet-stream', 'cache-control': 'no-store' });
    fs.createReadStream(file).pipe(response);
  });
});

server.listen(port, '127.0.0.1', () => {
  console.log(`LUSI Rover Ops browser server: http://127.0.0.1:${port}/src/index.html`);
});
