// captive_pages.h
#pragma once

// Pagine HTML per il capti33ve portal (inserire qui, incluso come header).
// Metti questo file nella stessa cartella dello sketch .ino

// ---------- Captive pages (Apple-style, minimal & accessible) ----------
const char *CAPTIVE_PAGE_FORM = R"rawliteral(
<!doctype html>
<html lang="it">
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta charset="utf-8">
  <title>Air Volume — Configurazione Wi-Fi</title>
  <style>
    :root{
      --bg:#f2f4f7;
      --card:#ffffff;
      --muted:#6b6b6b;
      --accent:#007aff;
      --radius:14px;
      --shadow:0 8px 24px rgba(15,15,15,0.06);
      --maxw:460px;
      --pad:20px;
      --font:-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
    }

    html,body{height:100%;margin:0;background:var(--bg);font-family:var(--font);color:#111;-webkit-font-smoothing:antialiased;}
    .frame{min-height:100vh;display:flex;align-items:center;justify-content:center;padding:28px;box-sizing:border-box;}
    .card{width:100%;max-width:var(--maxw);background:var(--card);border-radius:var(--radius);box-shadow:var(--shadow);padding:var(--pad);box-sizing:border-box;}
    header{display:flex;align-items:center;gap:12px;margin-bottom:10px;}
    .logo{width:44px;height:44px;border-radius:10px;background:linear-gradient(135deg,#f7f8fa,#eef3ff);display:flex;align-items:center;justify-content:center;font-weight:700;color:var(--accent);font-size:20px;}
    h1{margin:0;font-size:17px;font-weight:600;}
    p.lead{margin:6px 0 14px 0;color:var(--muted);font-size:13px;line-height:1.35;}
    form{display:block;}
    label{display:block;font-size:12px;color:#444;margin-bottom:6px;font-weight:600;}
    input[type=text], input[type=password]{
      width:100%;padding:12px;border-radius:10px;border:1px solid rgba(0,0,0,0.08);background:#fbfbfd;font-size:15px;box-sizing:border-box;
    }
    input[type=text]:focus, input[type=password]:focus{
      outline:none;box-shadow:0 0 0 6px rgba(0,122,255,0.06);border-color:rgba(0,122,255,0.6);background:#fff;
    }
    .row{margin-bottom:12px;}
    .actions{display:flex;gap:10px;align-items:center;margin-top:6px;}
    button.primary{
      -webkit-appearance:none;appearance:none;border:0;background:var(--accent);color:#fff;padding:10px 14px;border-radius:12px;font-weight:700;font-size:15px;cursor:pointer;
      box-shadow:0 6px 14px rgba(0,122,255,0.14);
    }
    button.ghost{background:transparent;border:0;color:var(--accent);font-weight:600;cursor:pointer;}
    .hint{margin-top:12px;font-size:12px;color:var(--muted);}
    a.simple{color:var(--accent);text-decoration:none;font-weight:600;}
    footer{margin-top:14px;font-size:12px;color:var(--muted);}
    @media (max-width:480px){ .card{padding:16px;border-radius:12px;} .logo{width:40px;height:40px} }
  </style>
</head>
<body>
  <div class="frame">
    <main class="card" role="main" aria-labelledby="formTitle">
      <header>
        <div class="logo" aria-hidden="true">AV</div>
        <div>
          <h1 id="formTitle">Air Volume</h1>
          <p class="lead">Configura la rete Wi-Fi del dispositivo in modo semplice e sicuro.</p>
        </div>
      </header>

      <form method="POST" action="/save" aria-describedby="desc">
        <div id="desc" class="hint">Inserisci SSID e, se necessario, la password della rete.</div>

        <div class="row">
          <label for="ssid">SSID</label>
          <input id="ssid" name="ssid" type="text" required minlength="1" maxlength="63" autocomplete="ssid" inputmode="text" aria-required="true">
        </div>

        <div class="row">
          <label for="pass">Password (opzionale)</label>
          <input id="pass" name="pass" type="password" maxlength="63" autocomplete="current-password" inputmode="text">
        </div>

        <div class="actions">
          <button type="submit" class="primary" aria-label="Connetti alla rete">Connetti</button>
          <a class="simple" href="/forget" role="link" aria-label="Rimuovi le credenziali salvate">Rimuovi credenziali</a>
        </div>

        <footer>
          <p class="hint">Se la connessione fallisce, l'access point verrà riattivato automaticamente.</p>
        </footer>
      </form>
    </main>
  </div>
</body>
</html>
)rawliteral";

const char *CAPTIVE_PAGE_CONNECTING = R"rawliteral(
<!doctype html>
<html lang="it">
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta charset="utf-8">
  <title>Air Volume — Collegamento</title>
  <style>
    :root{
      --bg:#f2f4f7;
      --card:#ffffff;
      --muted:#6b6b6b;
      --accent:#007aff;
      --radius:14px;
      --shadow:0 8px 24px rgba(15,15,15,0.06);
      --maxw:420px;
      --pad:20px;
      --font:-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
    }
    html,body{height:100%;margin:0;background:var(--bg);font-family:var(--font);color:#111;}
    .frame{min-height:100vh;display:flex;align-items:center;justify-content:center;padding:28px;box-sizing:border-box;}
    .card{width:100%;max-width:var(--maxw);background:var(--card);border-radius:var(--radius);box-shadow:var(--shadow);padding:var(--pad);box-sizing:border-box;text-align:center;}
    h1{margin:0;font-size:18px;font-weight:600;}
    p{margin:12px 0;color:var(--muted);font-size:14px;line-height:1.35;}
    .spinner{width:56px;height:56px;border-radius:50%;border:5px solid rgba(0,0,0,0.06);border-top-color:var(--accent);margin:18px auto;animation:spin 1s linear infinite;}
    @keyframes spin{to{transform:rotate(360deg);} }
    .small{font-size:13px;color:var(--muted);}
    @media (max-width:480px){ .card{padding:16px;} .spinner{width:48px;height:48px;} }
  </style>
</head>
<body>
  <div class="frame">
    <main class="card" role="status" aria-live="polite" aria-atomic="true">
      <h1>Collegamento in corso…</h1>
      <div class="spinner" aria-hidden="true"></div>
      <p>Il dispositivo sta tentando di connettersi alla rete Wi-Fi. Se l'operazione impiega troppo tempo, riapri la rete <strong>Air Volume</strong> per riprovare.</p>
      <p class="small">Puoi chiudere questa pagina: il processo continua autonomamente.</p>
    </main>
  </div>
</body>
</html>
)rawliteral";

const char *CAPTIVE_PAGE_FORGOTTEN = R"rawliteral(
<!doctype html>
<html lang="it">
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta charset="utf-8">
  <title>Air Volume — Credenziali rimosse</title>
  <style>
    :root{
      --bg:#f2f4f7;
      --card:#ffffff;
      --muted:#6b6b6b;
      --accent:#007aff;
      --radius:14px;
      --shadow:0 8px 24px rgba(15,15,15,0.06);
      --maxw:420px;
      --pad:20px;
      --font:-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
    }
    html,body{height:100%;margin:0;background:var(--bg);font-family:var(--font);color:#111;}
    .frame{min-height:100vh;display:flex;align-items:center;justify-content:center;padding:28px;box-sizing:border-box;}
    .card{width:100%;max-width:var(--maxw);background:var(--card);border-radius:var(--radius);box-shadow:var(--shadow);padding:var(--pad);box-sizing:border-box;text-align:left;}
    h1{margin:0 0 6px 0;font-size:18px;font-weight:600;}
    p{margin:8px 0;color:var(--muted);font-size:14px;line-height:1.35;}
    a{color:var(--accent);text-decoration:none;font-weight:700;}
    @media (max-width:480px){ .card{padding:16px;} }
  </style>
</head>
<body>
  <div class="frame">
    <main class="card" role="main" aria-labelledby="forgotTitle">
      <h1 id="forgotTitle">Credenziali rimosse</h1>
      <p>Le credenziali salvate sul dispositivo sono state cancellate correttamente.</p>
      <p class="small">Ricollegati all'access point <strong>Air Volume</strong> o ricarica questa pagina per inserire nuove informazioni.</p>
      <p style="margin-top:12px;"><a href="/" aria-label="Torna alla pagina iniziale">Torna alla pagina iniziale</a></p>
    </main>
  </div>
</body>
</html>
)rawliteral";