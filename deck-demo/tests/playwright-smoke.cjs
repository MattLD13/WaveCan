const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const { URL } = require('node:url');
const root = path.resolve(__dirname, '..', 'src');
const requireBrowser = process.env.REQUIRE_BROWSER === '1';
let playwright;
try { playwright = require(process.env.PLAYWRIGHT_MODULE || 'playwright'); } catch (error) {
  console.error(`Playwright package unavailable: ${error.message}`); process.exit(requireBrowser ? 1 : 0);
}
const mime = { '.html':'text/html', '.js':'text/javascript', '.css':'text/css' };
const server = http.createServer((req,res)=>{ const url = new URL(req.url,'http://localhost'); const file = path.resolve(root, '.' + url.pathname); if (!file.startsWith(root)) { res.writeHead(403); return res.end(); } fs.readFile(file,(err,data)=>{ if(err){res.writeHead(404);return res.end();} res.writeHead(200,{'content-type':mime[path.extname(file)]||'text/plain'});res.end(data); }); });
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  const port = server.address().port;
  let browser;
  try {
    browser = await playwright.chromium.launch({headless:true});
    const page = await browser.newPage({viewport:{width:1280,height:800}});
    await page.goto(`http://127.0.0.1:${port}/index.html`);
    await page.waitForSelector('#camera-canvas');
    if (await page.title() !== 'LUSI Rover Ops') throw new Error('unexpected title');
    await page.locator('#demo-button').click();
    await page.waitForFunction(() => document.querySelector('#demo-status')?.textContent.includes('STEP 1'));
    await page.keyboard.down('Space'); await page.keyboard.down('w'); await page.waitForTimeout(130);
    if (await page.locator('#deadman-badge').textContent() !== 'DEADMAN ON') throw new Error('deadman did not engage');
    await page.keyboard.up('w'); await page.keyboard.up('Space'); await page.waitForTimeout(80);
    if (await page.locator('#deadman-badge').textContent() !== 'HOLD TO ENABLE') throw new Error('deadman did not release');
    await page.locator('[data-camera="arm"]').click();
    if (await page.locator('#camera-name').textContent() !== 'ARM CAM') throw new Error('camera action failed');
    await page.locator('#sim-link-toggle').click();
    if (await page.locator('#connection-label').textContent() !== 'LINK LOST') throw new Error('link loss action failed');
    if (!await page.locator('#no-downlink-image').evaluate((node) => node.classList.contains('visible'))) throw new Error('no-downlink state not visible');
    const output = process.env.PLAYWRIGHT_OUTPUT || path.resolve(__dirname, '..', 'previews', 'playwright-smoke.png');
    await page.screenshot({path:output});
    console.log(`PLAYWRIGHT_SMOKE_OK ${output}`);
  } catch (error) {
    if (!requireBrowser && /Executable doesn't exist|Please run.*playwright install/i.test(String(error))) {
      console.warn('PLAYWRIGHT_SMOKE_SKIPPED: Chromium is not installed; set REQUIRE_BROWSER=1 to make this a hard gate.');
    } else {
      console.error(`PLAYWRIGHT_SMOKE_FAILED ${error.stack || error}`); process.exitCode=1;
    }
  } finally { if(browser) await browser.close(); server.close(); }
})();
