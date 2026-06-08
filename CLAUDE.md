# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Static HTML portfolio website (个人开发作品集) by Ezebr. No build tools, no package manager, no tests — just HTML/CSS/JS served as static files (e.g., via Nginx).

## Architecture

- **index.html** — Landing page with hero, project cards grid, skills, about, and contact sections.
- **project-*.html** — Individual project detail pages, one per project. Each shares the same navbar and footer pattern from index.html.
- **css/style.css** — Global styles (navbar, hero, sections, responsive breakpoints).
- **css/project.css** — Styles specific to project detail pages (blog-post layout, SVG diagrams, tech badges).
- **js/main.js** — Mouse glow effect, mobile hamburger menu toggle, navbar scroll shadow, and scroll-triggered fade-in animations (IntersectionObserver).
- **images/** — SVG architecture diagrams for project pages.
- **doc/** — Markdown project documentation (not served directly; content is embedded in HTML pages).

## Conventions

- Language is `zh-CN` throughout (Chinese content, Chinese meta descriptions).
- Icons via Phosphor Icons CDN (`<script src="https://unpkg.com/@phosphor-icons/web">`), used as `<i class="ph ph-*">`.
- Project detail pages follow a consistent blog-post structure: hero banner → overview → architecture diagram (SVG) → technical deep-dive sections → tech stack badges.
- Responsive design uses CSS media queries (breakpoints at 768px and 480px).
- Color scheme: dark background (#0a0a0a), accent blue (#3b82f6), card backgrounds (#111).

## Adding a New Project

1. Copy an existing `project-*.html` as a template.
2. Add a project card to the `projects-grid` in `index.html`.
3. Place any SVG diagrams in `images/`.
4. Place any markdown docs in `doc/`.
