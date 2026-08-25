import { defineConfig } from "vitepress";

const github = "https://github.com/Bharath2/fx2d-physics-engine";

export default defineConfig({
  title: "Fx2D",
  description: "A C++20 2D rigid-body physics engine.",
  base: "/fx2d-physics-engine/",
  cleanUrls: true,
  lastUpdated: true,
  ignoreDeadLinks: [
    /localhost/,
    /github\.com/,
    /cpp-api\//,
  ],
  head: [
    ["meta", { name: "theme-color", content: "#071c2b" }],
    ["meta", { property: "og:type", content: "website" }],
    ["meta", { property: "og:title", content: "Fx2D Physics Engine" }],
    [
      "meta",
      {
        property: "og:description",
        content: "A C++20 2D rigid-body engine built around SAT and XPBD.",
      },
    ],
  ],
  themeConfig: {
    logo: "/mark.svg",
    siteTitle: "Fx2D",
    nav: [
      { text: "Guides", link: "/getting-started/install" },
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
            { text: "Scene authoring", link: "/reference/scene-yaml" },
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
            { text: "Scene YAML", link: "/reference/scene-yaml" },
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
