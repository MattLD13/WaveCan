const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const { URL } = require('node:url');

const root = path.resolve(__dirname, '..');
const requireBrowser = process.env.REQUIRE_BROWSER === '1';
let playwright;
try { playwright = require(process.env.PLAYWRIGHT_MODULE || 'playwright'); } catch (error) {
  console.error(`Playwright package unavailable: ${error.message}`); process.exit(requireBrowser ? 1 : 0);
}
const mime = { '.html': 'text/html', '.js': 'text/javascript', '.cjs': 'text/javascript', '.css': 'text/css', '.png': 'image/png', '.svg': 'image/svg+xml' };
const server = http.createServer((req, res) => {
  const url = new URL(req.url, 'http://localhost');
  const file = path.resolve(root, `.${url.pathname}`);
  if (!file.startsWith(root)) { res.writeHead(403); return res.end(); }
  fs.readFile(file, (error, data) => {
    if (error) { res.writeHead(404); return res.end(); }
    res.writeHead(200, { 'content-type': mime[path.extname(file)] || 'application/octet-stream' });
    res.end(data);
  });
});

(async () => {
  await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
  const port = server.address().port;
  let browser;
  try {
    browser = await playwright.chromium.launch({ headless: true, args: ['--use-gl=swiftshader', '--no-sandbox'] });
    const page = await browser.newPage({ viewport: { width: 1280, height: 800 }, deviceScaleFactor: 1 });
    const consoleErrors = [];
    page.on('console', (message) => { if (message.type() === 'error') consoleErrors.push(message.text()); });
    await page.goto(`http://127.0.0.1:${port}/src/index.html`);
    await page.waitForSelector('#camera-canvas');
    if (await page.title() !== 'LUSI Mars Analog 01') throw new Error('unexpected title');
    await page.locator('#activate-button').click();
    await page.waitForFunction(() => document.querySelector('#mission-phase-label')?.textContent.includes('APPROACH'));
    await page.keyboard.down('Space'); await page.keyboard.down('w'); await page.waitForTimeout(160);
    if (await page.locator('#deadman-badge').textContent() !== 'DEADMAN ON') throw new Error('deadman did not engage');
    await page.keyboard.up('w'); await page.keyboard.up('Space'); await page.waitForTimeout(100);
    if (await page.locator('#deadman-badge').textContent() !== 'HOLD TO ENABLE') throw new Error('deadman did not release');
    const activeOutput = process.env.PLAYWRIGHT_ACTIVE_OUTPUT || path.resolve(__dirname, '..', 'previews', 'preview-mars-active-1280x800.png');
    await page.screenshot({ path: activeOutput });

    await page.evaluate(() => {
      const sim = window.__LUSI_SIM__;
      sim.state.pose = { x: 0.44, y: 0.47, heading: 43 };
      sim.tick(0);
    });
    await page.waitForFunction(() => document.querySelector('#science-park-status')?.textContent === 'OUTCROP PARKED');
    const scienceOutput = process.env.PLAYWRIGHT_SCIENCE_OUTPUT || path.resolve(__dirname, '..', 'previews', 'preview-mars-science-1280x800.png');
    await page.screenshot({ path: scienceOutput });
    for (const action of ['geo-camera', 'classify-rock', 'deep-sample', 'discard-shallow', 'load-cell', 'soil-capture', 'hydrogen-peroxide', 'cobalt-bicarbonate', 'blank-cuvette', 'control-sample', 'sample-reading']) {
      await page.evaluate((scienceAction) => window.__LUSI_SIM__.performScience(scienceAction), action);
      await page.waitForTimeout(30);
    }
    const assayOutput = process.env.PLAYWRIGHT_ASSAY_OUTPUT || path.resolve(__dirname, '..', 'previews', 'preview-mars-assay-1280x800.png');
    await page.screenshot({ path: assayOutput });
    await page.evaluate(() => {
      const sim = window.__LUSI_SIM__;
      sim.state.pose = { x: 0.73, y: 0.27, heading: 43 };
      sim.tick(0);
    });
    await page.waitForFunction(() => document.querySelector('#beacon-module')?.hidden === false);
    await page.evaluate(() => window.__LUSI_SIM__.performBeacon('run-macro'));
    await page.waitForTimeout(900);
    const beaconOutput = process.env.PLAYWRIGHT_BEACON_OUTPUT || path.resolve(__dirname, '..', 'previews', 'preview-mars-beacon-1280x800.png');
    await page.screenshot({ path: beaconOutput });
    if (consoleErrors.length) throw new Error(`browser console errors: ${consoleErrors.join('; ')}`);
    console.log(`PLAYWRIGHT_SMOKE_OK ${activeOutput} ${scienceOutput} ${assayOutput} ${beaconOutput}`);
  } catch (error) {
    if (!requireBrowser && /Executable doesn't exist|Please run.*playwright install/i.test(String(error))) {
      console.warn('PLAYWRIGHT_SMOKE_SKIPPED: Chromium is not installed; set REQUIRE_BROWSER=1 to make this a hard gate.');
    } else {
      console.error(`PLAYWRIGHT_SMOKE_FAILED ${error.stack || error}`); process.exitCode = 1;
    }
  } finally {
    if (browser) await browser.close();
    server.close();
  }
})();
