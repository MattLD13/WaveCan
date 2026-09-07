from PIL import Image, ImageDraw, ImageFont
from pathlib import Path

OUT = Path(__file__).parent / "previews"; OUT.mkdir(exist_ok=True)
W, H = 1280, 800
BG, PANEL, LINE = "#071019", "#0e2029", "#21404b"
INK, DIM, CYAN, GREEN, AMBER, RED = "#e6f1f5", "#86a2ad", "#61e7ef", "#7bea9a", "#f6bd60", "#ff6b6b"
def font(size, bold=False):
    p = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(p, size) if p.exists() else ImageFont.load_default()
def txt(d, xy, s, size=12, color=INK, bold=False, anchor=None): d.text(xy, s, font=font(size,bold), fill=color, anchor=anchor)
def box(d, xy, r=8, fill=PANEL, outline=LINE): d.rounded_rectangle(xy, radius=r, fill=fill, outline=outline, width=1)
def base(d, alt=False):
    d.rectangle((0,0,W,H), fill=BG); d.rectangle((0,0,W,64), fill="#08141d"); d.line((0,63,W,63), fill=LINE)
    txt(d,(24,28),"◈",25,CYAN,True,"lm"); txt(d,(53,26),"LUSI",18,INK,True,"lm"); txt(d,(111,26),"//OPS",18,DIM,True,"lm")
    d.rounded_rectangle((229,17,298,38),radius=4,fill="#17282c",outline="#765b36"); txt(d,(263,28),"SIM ONLY",10,AMBER,True,"mm")
    txt(d,(640,15),"MISSION",10,DIM,True,"mm"); txt(d,(640,35),"LUSI ROVER / COURSE 01",12,INK,True,"mm")
    d.ellipse((1015,26,1024,35),fill=CYAN); txt(d,(1031,15),"SIM LINK",12,INK,True); txt(d,(1031,34),"42 ms • 60 Hz",10,DIM)
    d.rounded_rectangle((1175,23,1194,33),radius=2,outline=GREEN); d.rectangle((1177,25,1191,31),fill=GREEN); txt(d,(1207,28),"94%",12,INK,True,"lm")
    d.rectangle((0,64,W,69),fill="#091820"); d.line((0,68,W,68),fill=LINE)
    colors=[CYAN,AMBER,RED if alt else GREEN,RED if alt else (195,139,255),(110,168,255)]
    for i,color in enumerate(colors): d.rectangle((i*W/5,64,(i+1)*W/5-1,68),fill=color)
def camera(d,x,y,w,h,alt=False):
    box(d,(x,y,x+w,y+h),5,fill="#0a1820",outline="#244954"); d.rectangle((x+1,y+1,x+w-1,y+h-1),fill="#153b44")
    horizon=y+h*.42; d.rectangle((x+1,y+h*.42,x+w-1,y+h-1),fill="#285044")
    d.polygon([(x,y+h*.42+22),(x+w*.16,y+h*.42-4),(x+w*.32,y+h*.42+17),(x+w*.51,y+h*.42-12),(x+w*.68,y+h*.42+14),(x+w*.84,y+h*.42-2),(x+w,y+h*.42+20),(x+w,y+h),(x,y+h)],fill="#35695a")
    for yy in range(int(horizon+42),y+h,36): d.line((x,yy,x+w,yy),fill="#3b7168")
    for px,py,rr in ((.18,.67,18),(.74,.63,26),(.56,.51,13),(.84,.78,34),(.32,.82,22)): d.ellipse((x+px*w-rr,y+py*h-rr,x+px*w+rr,y+py*h+rr),fill="#c8a36a")
    d.ellipse((x+w/2-14,y+h*.56-14,x+w/2+14,y+h*.56+14),outline="#d8f7f799"); d.line((x+w/2-23,y+h*.56,x+w/2+23,y+h*.56),fill="#d8f7f799"); d.line((x+w/2,y+h*.56-23,x+w/2,y+h*.56+23),fill="#d8f7f799")
    d.rectangle((x,y+h-24,x+w,y+h),fill="#071019"); txt(d,(x+13,y+11),"● LUSI VISION / SIM FEED",10,INK,True); txt(d,(x+w-13,y+11),"1080p • 30 FPS",10,DIM,True,"ra"); txt(d,(x+12,y+h-9),"FRONT CAMERA  •  30 FPS",8,INK,True)
    d.rounded_rectangle((x+12,y+h-32,x+153,y+h-12),radius=3,fill=BG,outline="#9aeef077"); txt(d,(x+82,y+h-22),"FRONT CLEARANCE",8,INK,True,"mm")
    if alt:
        no_downlink = Image.open(Path(__file__).parent / "assets" / "LUSNoDownlink.png").convert("RGB").resize((w,h))
        d._image.paste(no_downlink, (x, y))
def mapview(d,x,y,w,h):
    box(d,(x,y,x+w,y+h),4,fill="#0c1b23",outline="#1e3c46")
    for xx in range(x,x+w,24): d.line((xx,y,xx,y+h),fill=LINE)
    for yy in range(y,y+h,24): d.line((x,yy,x+w,yy),fill=LINE)
    pts=[(x+w*.12,y+h*.76),(x+w*.28,y+h*.65),(x+w*.31,y+h*.32),(x+w*.52,y+h*.42),(x+w*.68,y+h*.55),(x+w*.66,y+h*.2),(x+w*.88,y+h*.24)]
    d.line(pts,fill=AMBER,width=3,joint="curve"); d.ellipse((pts[-1][0]-6,pts[-1][1]-6,pts[-1][0]+6,pts[-1][1]+6),fill=GREEN)
    d.polygon([(x+w*.12+8,y+h*.76),(x+w*.12-5,y+h*.76-7),(x+w*.12-5,y+h*.76+7)],fill=CYAN); txt(d,(x+8,y+h-12),"START",8,DIM); txt(d,(x+w-100,y+12),"CHECKPOINT",8,GREEN)
def render(path, alt=False, demo=False):
    im=Image.new("RGB",(W,H),BG); d=ImageDraw.Draw(im); base(d,alt)
    # A centered 16:9 camera surface with a slim utility rail on the right.
    box(d,(64,78,1066,594)); txt(d,(80,93),"LUSI VISION",10,DIM,True); txt(d,(80,109),"ARM CAM" if alt else "FRONT CAM",15,INK,True)
    for i,(lab,lx) in enumerate((("FRONT",834),("REAR",886),("ARM",936),("OVERHEAD",986))):
        active=(alt and lab=="ARM") or (not alt and lab=="FRONT"); ww=43+(30 if lab=="OVERHEAD" else 0)
        d.rounded_rectangle((lx,91,lx+ww,119),radius=4,fill="#1a4c57" if active else "#142a33",outline="#3d9da6" if active else "#244956"); txt(d,(lx+ww/2,105),lab,8,CYAN if active else DIM,True,"mm")
    camera(d,124,128,880,456,alt)
    for i,(lab,val) in enumerate((("FIELD OF VIEW","78°"),("MODE","DRIVE ACTIVE" if demo else "READY / HOLD TO ENABLE"),("LINK AGE","0 ms"))): txt(d,(124+i*292,594),lab,8,DIM,True); txt(d,(124+i*292+95,594),val,8,INK,True)
    box(d,(124,612,552,728),7); txt(d,(138,625),"ROVER TELEMETRY",9,DIM,True); txt(d,(530,625),"● LIVE",9,GREEN,True,"ra")
    for i,(lab,val) in enumerate((("SPEED","0.32 m/s" if demo else "0.00 m/s"),("HEADING","018°"),("DISTANCE","0.15 m" if demo else "0.00 m"))): xx=138+i*126; txt(d,(xx,649),lab,8,DIM,True); txt(d,(xx,664),val,16,INK,True); d.rounded_rectangle((xx,694,xx+96,698),2,fill="#1a3540"); d.rectangle((xx,694,xx+(70 if demo and i==0 else 0),698),fill=CYAN)
    box(d,(566,612,1004,728),7); txt(d,(580,625),"MISSION PHASE",9,DIM,True); txt(d,(990,625),"STEP 2 / 4" if demo else "READY",9,AMBER,True,"ra")
    txt(d,(580,648),"CONNECT  ›  DRIVE  ›  CAMERA  ›  CHECKPOINT",9,INK,True); txt(d,(580,669),"THROTTLE SETS SPEED • HOLD TO ENABLE",8,AMBER,True)
    d.rounded_rectangle((580,689,990,714),radius=3,fill="#165663"); txt(d,(785,701),"START GUIDED DEMO     H",9,INK,True,"mm")
    # Slim right rail: map, one compact control model, and recovery state.
    box(d,(1084,110,1262,245),7); txt(d,(1095,123),"COURSE MAP",9,DIM,True); txt(d,(1251,123),"● ROVER",8,DIM,True,"ra"); mapview(d,1095,138,156,92); txt(d,(1095,242),"POS  0.12 / 0.48",8,INK,True)
    box(d,(1084,255,1262,514),7); txt(d,(1095,268),"DRIVE CONTROL",9,DIM,True); txt(d,(1251,268),"DEADMAN ON" if demo else "HOLD TO ENABLE",8,GREEN if demo else RED,True,"ra")
    for i,(lab,val) in enumerate((("THROTTLE","0.40" if demo else "0.00"),("TURN","0.00"))): xx=1095+i*77; d.rounded_rectangle((xx,286,xx+70,345),radius=3,fill="#102731",outline="#1c3d48"); txt(d,(xx+6,298),lab,7,DIM,True); txt(d,(xx+6,318),val,15,INK,True); d.rounded_rectangle((xx+6,335,xx+64,339),2,fill="#1a3540"); d.rectangle((xx+6,335,xx+(35 if demo and i==0 else 6),339),fill=AMBER)
    txt(d,(1095,361),"THROTTLE = SPEED",8,AMBER,True); txt(d,(1095,374),"FORWARD / REVERSE",7,DIM,True)
    txt(d,(1173,400),"▲",18,CYAN,True,"mm"); txt(d,(1127,426),"◀",18,CYAN,True,"mm"); d.rounded_rectangle((1150,411,1196,441),radius=3,fill="#3c2730",outline="#814148"); txt(d,(1173,426),"■",12,RED,True,"mm"); txt(d,(1219,426),"▶",18,CYAN,True,"mm"); txt(d,(1173,453),"▼",18,CYAN,True,"mm")
    d.line((1095,476,1251,476),fill=LINE); txt(d,(1095,490),"L  WASD / LEFT STICK",7,DIM,True); txt(d,(1251,490),"SPACE / R2",7,AMBER,True,"ra")
    box(d,(1084,524,1262,674),7); txt(d,(1095,537),"SAFETY & LINK",9,DIM,True); txt(d,(1251,537),"SIM ONLY",8,AMBER,True,"ra")
    for i,label in enumerate(("SIMULATED LINK","COMMAND WATCHDOG")): yy=563+i*23; d.ellipse((1096,yy,1103,yy+7),fill=CYAN if i==0 else GREEN); txt(d,(1111,yy+4),label,8,INK,True,"lm"); txt(d,(1251,yy+4),"ON" if i==0 else "100 ms",8,GREEN,True,"rm")
    d.rounded_rectangle((1095,617,1251,646),radius=3,fill=RED if alt else "#532d34",outline=RED); txt(d,(1173,631),"CLEAR STOP" if alt else "STOP OUTPUT",9,"#270d10" if alt else "#ffb4b4",True,"mm"); txt(d,(1173,662),"LINK LOST • OUTPUT ZEROED" if alt else "DRIVE COMMAND ACTIVE" if demo else "SIMULATION READY",7,AMBER,True,"mm")
    d.rectangle((0,782,W,H),fill="#08141d"); d.line((0,782,W,782),fill=LINE); txt(d,(24,792),"A  DEMO / SELECT     B  STOP     X  CAMERA     Y  MAP",9,DIM,True,"lm"); txt(d,(1255,792),"● INPUT READY   │   ESC  EMERGENCY STOP",9,DIM,True,"rm")
    im.save(path)
def render_deck_mockup(path):
    im=Image.new("RGB",(W,H),"#050b10"); d=ImageDraw.Draw(im)
    txt(d,(640,34),"LUSI ROVER OPS  /  1280 × 800 LCD CONTROL SURFACE",15,INK,True,"mm")
    d.rounded_rectangle((80,155,1200,670),radius=52,fill="#1c2228",outline="#5e6b72",width=3)
    d.rounded_rectangle((99,174,1181,651),radius=42,fill="#11191f",outline="#303d44",width=2)
    # The official Valve front outline is provided in assets/steamdeckFront.svg; this clean preview uses the same front-on LCD proportions.
    d.rounded_rectangle((322,223,958,584),radius=12,fill="#020609",outline="#76858b",width=2)
    camera(d,330,231,620,345,False)
    # original LCD day-one front controls: left stick/d-pad, right stick/face cluster, shoulders and grips
    d.ellipse((166,311,246,391),fill="#0c1115",outline="#6d7a80",width=2); d.ellipse((186,331,226,371),fill="#273139",outline="#87959b")
    d.ellipse((1025,311,1105,391),fill="#0c1115",outline="#6d7a80",width=2); d.ellipse((1045,331,1085,371),fill="#273139",outline="#87959b")
    for x,y,label in ((260,330,"+") ,(260,372,"B"),(1004,330,"Y"),(1004,372,"A")): d.ellipse((x-16,y-16,x+16,y+16),fill="#27343b",outline=CYAN); txt(d,(x,y),label,12,CYAN,True,"mm")
    txt(d,(640,705),"ORIGINAL LCD / 512 GB DAY-ONE SILHOUETTE REFERENCE",11,DIM,True,"mm")
    txt(d,(640,733),"Official front line art: Steamworks Steam Deck SVG Line Art",10,DIM,False,"mm")
    im.save(path)
render(OUT/"preview-drive-1280x800.png")
render(OUT/"preview-arm-linklost-1280x800.png",alt=True)
render(OUT/"preview-guided-demo-1280x800.png",demo=True)
render_deck_mockup(OUT/"preview-lusi-steamdeck-lcd-front.png")
