import { defineConfig } from "vitepress";

const github = "https://github.com/Bharath2/fx2d-physics-engine";
const site = "https://bharath2.github.io/fx2d-physics-engine/";
const description =
  "Fx2D is an open-source 2D rigid-body physics engine in C++20: SAT collision detection, an XPBD constraint solver, motorized joints, YAML scenes, and a headless mode for simulation, testing and reinforcement learning.";

export default defineConfig({
  lang: "en-US",
  title: "Fx2D",
  titleTemplate: ":title · Fx2D 2D Physics Engine",
  description,
  base: "/fx2d-physics-engine/",
  cleanUrls: true,
  lastUpdated: true,
  sitemap: { hostname: site },
  ignoreDeadLinks: [
    /localhost/,
    /github\.com/,
    /cpp-api\//,
  ],
  head: [
    ["meta", { name: "theme-color", content: "#071c2b" }],
    [
      "meta",
      {
        name: "keywords",
        content:
          "2D physics engine, C++ physics engine, C++20, rigid body dynamics, XPBD, position based dynamics, SAT collision detection, separating axis theorem, game physics, physics simulation, headless simulation, reinforcement learning environment, raylib, Box2D alternative, open source",
      },
    ],
    ["meta", { name: "author", content: "Bharath Chandra Irigireddy" }],
    ["meta", { name: "robots", content: "index, follow" }],
    ["link", { rel: "icon", href: "/fx2d-physics-engine/mark.svg", type: "image/svg+xml" }],
    ["meta", { property: "og:type", content: "website" }],
    ["meta", { property: "og:site_name", content: "Fx2D Physics Engine" }],
    ["meta", { property: "og:title", content: "Fx2D — 2D rigid-body physics engine in C++20" }],
    ["meta", { property: "og:description", content: description }],
    ["meta", { property: "og:url", content: site }],
    ["meta", { property: "og:image", content: `${site}social-card.png` }],
    ["meta", { property: "og:image:width", content: "1200" }],
    ["meta", { property: "og:image:height", content: "630" }],
    ["meta", { name: "twitter:card", content: "summary_large_image" }],
    ["meta", { name: "twitter:title", content: "Fx2D — 2D rigid-body physics engine in C++20" }],
    ["meta", { name: "twitter:description", content: description }],
    ["meta", { name: "twitter:image", content: `${site}social-card.png` }],
    [
      "script",
      { type: "application/ld+json" },
      JSON.stringify({
        "@context": "https://schema.org",
        "@type": "SoftwareSourceCode",
        name: "Fx2D",
        description,
        url: site,
        codeRepository: github,
        programmingLanguage: "C++",
        runtimePlatform: "C++20",
        license: "https://opensource.org/licenses/BSD-3-Clause",
        applicationCategory: "DeveloperApplication",
        keywords:
          "2D physics engine, rigid body dynamics, XPBD, SAT collision detection, C++20, game physics, simulation",
        author: { "@type": "Person", name: "Bharath Chandra Irigireddy", url: "https://github.com/Bharath2" },
      }),
    ],
  ],
  transformPageData(pageData) {
    const path = pageData.relativePath
      .replace(/(^|\/)index\.md$/, "$1")
      .replace(/\.md$/, "");
    const canonical = site + path;
    pageData.frontmatter.head ??= [];
    pageData.frontmatter.head.push(["link", { rel: "canonical", href: canonical }]);
    if (pageData.frontmatter.description) {
      pageData.frontmatter.head.push([
        "meta",
        { property: "og:description", content: pageData.frontmatter.description },
      ]);
    }
  },
  themeConfig: {
    logo: "/mark.svg",
    siteTitle: "Fx2D",
    nav: [
      { text: "Guides", link: "/guides/" },
      { text: "Demos", link: "/demos" },
      { text: "API", link: "/api/" },
      { text: "GitHub", link: github },
    ],
    sidebar: {
      "/getting-started/": [
        {
          text: "Get started",
          items: [
            { text: "Installation", link: "/getting-started/install" },
            { text: "Your first scene", link: "/getting-started/first-scene" },
          ],
        },
      ],
      "/guides/": [
        {
          text: "Build with Fx2D",
          items: [
            { text: "Scene authoring", link: "/guides/scene-authoring" },
            { text: "Math and geometry", link: "/guides/math" },
            { text: "Collision setup", link: "/guides/collisions" },
            { text: "Constraints", link: "/guides/constraints" },
            { text: "Headless simulation", link: "/guides/headless" },
            { text: "Renderer", link: "/guides/renderer" },
            { text: "Input", link: "/guides/input" },
            { text: "Joints and motors", link: "/guides/joints" },
            { text: "Queries", link: "/guides/queries" },
            { text: "Contacts and sensors", link: "/guides/events" },
            { text: "Entity groups", link: "/guides/entity-groups" },
          ],
        },
      ],
      "/concepts/": [
        {
          text: "Engine concepts",
          items: [
            { text: "Collision pipeline", link: "/concepts/collisions" },
            { text: "XPBD solver", link: "/concepts/xpbd" },
          ],
        },
      ],
      "/reference/": [
        {
          text: "Reference",
          items: [
            { text: "Scene YAML specification", link: "/reference/scene-yaml" },
            { text: "Math utilities", link: "/reference/math" },
            { text: "C++ API", link: "/api/" },
          ],
        },
      ],
      "/api/": [
        {
          text: "C++ API",
          items: [
            { text: "Overview", link: "/api/" },
            { text: "Headers and entry points", link: "/api/headers" },
            { text: "Link Fx2Dlib", link: "/api/linking" },
            { text: "Headless simulation", link: "/api/headless" },
            { text: "Renderer integration", link: "/api/renderer" },
          ],
        },
        {
          text: "Generated reference",
          items: [
            { text: "All public symbols ↗", link: "/cpp-api/index.html" },
            { text: "FxScene ↗", link: "/cpp-api/classFxScene.html" },
            { text: "FxEntity ↗", link: "/cpp-api/classFxEntity.html" },
          ],
        },
      ],
      "/roadmap": [
        {
          text: "Project",
          items: [
            { text: "Roadmap", link: "/roadmap" },
            { text: "SIMD plan", link: "/roadmap/simd" },
            { text: "Fx3D plan", link: "/roadmap/fx3d" },
            { text: "Contributing", link: "/contributing" },
          ],
        },
      ],
    },
    socialLinks: [{ icon: "github", link: github }],
    editLink: {
      pattern: `${github}/edit/main/docs/:path`,
      text: "Edit this page on GitHub",
    },
    footer: {
      message: "Released under the BSD-3-Clause License.",
      copyright: "Copyright © 2026 Fx2D contributors",
    },
    search: {
      provider: "local",
    },
    outline: { level: [2, 3], label: "On this page" },
  },
});
