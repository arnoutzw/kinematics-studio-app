# Styling Guide — Maker Portfolio Apps

All apps must match the dark zinc/amber terminal-inspired theme. Apps are single HTML files, no build step. Load dependencies from CDN only.

## CDN Stack

```html
<script src="https://cdn.tailwindcss.com"></script>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500;600;700&display=swap" rel="stylesheet">
```

Configure Tailwind inline:
```html
<script>
tailwind.config = {
  theme: {
    extend: {
      fontFamily: {
        sans: ['Inter', 'sans-serif'],
        mono: ['JetBrains Mono', 'monospace']
      }
    }
  }
}
</script>
```

## Color Palette

| Role              | Tailwind Class          | Hex        |
|-------------------|-------------------------|------------|
| Page background   | `bg-zinc-950`           | `#09090b`  |
| Card / panel bg   | `bg-zinc-900`           | `#18181b`  |
| Inset / input bg  | `bg-zinc-800`           | `#27272a`  |
| Border default    | `border-zinc-700`       | `#3f3f46`  |
| Border accent     | `border-amber-500/50`   | —          |
| Text primary      | `text-zinc-100`         | `#f4f4f5`  |
| Text secondary    | `text-zinc-400`         | `#a1a1aa`  |
| Text muted        | `text-zinc-500`         | `#71717a`  |
| Accent primary    | `text-amber-500` / `bg-amber-500` | `#f59e0b` |
| Accent hover      | `text-amber-400` / `bg-amber-400` | `#fbbf24` |
| Accent subtle bg  | `bg-amber-500/10`       | —          |
| Danger            | `text-red-400` / `bg-red-500/20` | — |
| Success           | `text-green-400`        | —          |

## Typography

- **Body / UI**: `font-sans text-sm text-zinc-300`
- **Headings**: `font-mono font-bold text-zinc-100` (use `text-amber-500` for h1)
- **Labels / small text**: `text-xs text-zinc-500 font-mono`
- **Monospace data**: `font-mono` (values, code, IDs, hashes)

## Common Components

### Buttons
```
Primary:   bg-amber-500 hover:bg-amber-400 text-zinc-900 font-mono font-bold px-4 py-2.5 rounded-lg
Secondary: bg-zinc-700 hover:bg-zinc-600 text-zinc-300 text-xs font-mono px-3 py-1.5 rounded-lg
Ghost:     text-zinc-400 hover:text-amber-400 hover:bg-zinc-800 px-2 py-1.5 rounded-lg
```

### Inputs
```
bg-zinc-800 border border-zinc-700 rounded-lg px-4 py-2.5
font-mono text-sm text-zinc-100 placeholder:text-zinc-600
focus:border-amber-500 focus:outline-none
```

### Cards / Panels
```
bg-zinc-900 border border-zinc-700 rounded-lg p-5
hover:border-amber-500/50 hover:bg-zinc-800/80 hover:shadow-lg hover:shadow-amber-500/10
transition-all duration-300
```

### Terminal Window Header (decorative)
Three dots in a row:
```html
<div class="flex gap-1.5 mb-3">
  <div class="w-3 h-3 rounded-full bg-red-500"></div>
  <div class="w-3 h-3 rounded-full bg-yellow-500"></div>
  <div class="w-3 h-3 rounded-full bg-green-500"></div>
</div>
```

### Tags / Badges
```
text-xs px-2 py-0.5 bg-amber-500/10 text-amber-400 rounded font-mono
```

### Toggle Switch
```
Container: w-8 h-5 rounded-full (bg-amber-500 when on, bg-zinc-700 when off)
Dot:       w-4 h-4 rounded-full bg-white (left-3.5 when on, left-0.5 when off)
transition-all
```

## Layout

- Max content width: `max-w-7xl mx-auto`
- Grid: `grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6`
- Sticky header: `sticky top-0 z-40 bg-zinc-900/80 backdrop-blur-sm border-b border-zinc-800`
- Standard padding: `p-4` to `p-6` for sections, `p-5` for cards

## Scrollbars (add to `<style>`)
```css
::-webkit-scrollbar { width: 8px; height: 8px; }
::-webkit-scrollbar-track { background: #18181b; }
::-webkit-scrollbar-thumb { background: #3f3f46; border-radius: 4px; }
::-webkit-scrollbar-thumb:hover { background: #52525b; }
```

## Hover & Transitions

- Cards: `hover:-translate-y-1 transition-all duration-300`
- Buttons: `transition-colors`
- Accent glow on hover: `hover:shadow-lg hover:shadow-amber-500/10`
- Bottom bar reveal: `h-0.5 bg-gradient-to-r from-amber-500 to-orange-500 scale-x-0 group-hover:scale-x-100 transition-transform duration-300`

## Icons

Use Lucide icons via CDN (`lucide-react` for React apps, `lucide` for vanilla).
Standard sizes: 14-16px for inline, 18-20px for buttons, 32-48px for empty states.
Color: match surrounding text color (`text-zinc-400` default, `text-amber-500` for accent).

## Key Rules

1. Single HTML file — all CSS/JS inline
2. No npm, no build step — CDN only
3. `font-sans` (Inter) for UI, `font-mono` (JetBrains Mono) for data/code/headings
4. Zinc for neutrals, amber for accents — no other hue families except status colors (red/green/yellow)
5. Always include custom scrollbar styles
6. Backdrop blur on overlays: `bg-black/90 backdrop-blur-sm`
7. Rounded corners: `rounded-lg` (default), `rounded-xl` (modals)
