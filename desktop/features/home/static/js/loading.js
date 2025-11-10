// static/js/loading.js
(function () {
  const scene = document.querySelector('.scene');
  const robot = document.getElementById('robot');
  const eyes = document.querySelectorAll('.eye');
  const pupils = document.querySelectorAll('.pupil');
  const fill = document.getElementById('fill');
  const percent = document.getElementById('percent');
  const startBtn = document.getElementById('start-btn');
  const targetUrl = '/home';
  
  let p = 0;
  const tick = () => {
    const increment = p < 70 ? 4 + Math.random() * 6 : 1 + Math.random() * 3;
    p = Math.min(100, p + increment);
    fill.style.width = p.toFixed(0) + '%';
    percent.textContent = p.toFixed(0) + '%';

    if (p >= 100) {
      startBtn.classList.add('ready');
      startBtn.setAttribute('aria-hidden', 'false');
      clearInterval(timer);
    }
  };
  const timer = setInterval(tick, 200);

  // Eye tracking
  const maxOffset = 16; // max px pupil can move inside eye
  function movePupils(mx, my) {
    eyes.forEach((eye, i) => {
      const rect = eye.getBoundingClientRect();
      const cx = rect.left + rect.width / 2;
      const cy = rect.top + rect.height / 2;
      const dx = mx - cx;
      const dy = my - cy;
      const dist = Math.hypot(dx, dy) || 1;
      const ux = dx / dist;
      const uy = dy / dist;
      const px = ux * Math.min(maxOffset, dist * 0.15);
      const py = uy * Math.min(maxOffset, dist * 0.15);
      const pupil = pupils[i];
      pupil.style.transform = `translate(calc(-50% + ${px}px), calc(-50% + ${py}px))`;
    });
  }

  window.addEventListener('mousemove', (e) => {
    movePupils(e.clientX, e.clientY);

    // Subtle 3D tilt
    const vw = window.innerWidth;
    const vh = window.innerHeight;
    const rx = ((e.clientY / vh) - 0.5) * -10; // rotateX
    const ry = ((e.clientX / vw) - 0.5) * 12;  // rotateY
    robot.style.transform = `rotateX(${rx}deg) rotateY(${ry}deg)`;
  });

  // On touch screens, center gaze downward a bit
  window.addEventListener('touchmove', (e) => {
    const t = e.touches[0];
    movePupils(t.clientX, t.clientY);
  }, { passive: true });

  // Start navigation
  startBtn.addEventListener('click', () => {
    // Optional small bounce
    startBtn.style.transform = 'translateY(-2px) scale(0.98)';
    setTimeout(() => window.location.href = targetUrl, 150);
  });
})();
