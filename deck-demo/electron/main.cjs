const { app, BrowserWindow, Menu } = require('electron');
const path = require('node:path');

const createWindow = () => {
  const window = new BrowserWindow({
    width: 1280,
    height: 800,
    minWidth: 1024,
    minHeight: 640,
    title: 'LUSI Rover Ops',
    backgroundColor: '#071019',
    autoHideMenuBar: true,
    webPreferences: {
      contextIsolation: true,
      sandbox: true,
      nodeIntegration: false
    }
  });
  window.setMenuBarVisibility(false);
  window.loadFile(path.join(__dirname, '..', 'src', 'index.html'));
  if (process.env.LUSI_SMOKE === '1') {
    window.webContents.once('did-finish-load', async () => {
      const result = await window.webContents.executeJavaScript(`({ title: document.title, camera: Boolean(document.querySelector('#camera-canvas')), stop: Boolean(document.querySelector('#stop-button')) })`);
      if (result.title !== 'LUSI Rover Ops' || !result.camera || !result.stop) { console.error('ELECTRON_SMOKE_FAILED', result); app.exit(1); return; }
      console.log('ELECTRON_SMOKE_OK');
      app.exit(0);
    });
  }
};

app.whenReady().then(() => {
  Menu.setApplicationMenu(null);
  createWindow();
  app.on('activate', () => { if (BrowserWindow.getAllWindows().length === 0) createWindow(); });
});
app.on('window-all-closed', () => { if (process.platform !== 'darwin') app.quit(); });
