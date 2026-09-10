using System;
using System.Collections.Generic;
using UnityEngine;

public sealed class MarsWorldBuilder : MonoBehaviour
{
    public Transform BaseMarker { get; private set; }
    public Transform GeologyOutcropMarker { get; private set; }
    public Transform FieldMarker { get; private set; }
    public Transform FinalMarker { get; private set; }
    public Material RegolithMaterial { get; private set; }
    public Shader RuntimeRegolithShader { get; set; }

    private readonly List<GameObject> generatedObjects = new List<GameObject>();
    private Terrain foregroundTerrain;
    private Texture2D rockAlbedo;

    private const string HeightmapResource = "Mars/HadriacusPalusHeightmap";
    private const string AlbedoResource = "Mars/MarsTerrainAlbedo";
    private const string NormalResource = "Mars/MarsTerrainNormal";

    public void Build()
    {
        RenderSettings.fog = true;
        RenderSettings.fogMode = FogMode.ExponentialSquared;
        RenderSettings.fogColor = new Color(0.31f, 0.20f, 0.16f);
        RenderSettings.fogDensity = 0.0045f;
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
        RenderSettings.ambientSkyColor = new Color(0.30f, 0.20f, 0.18f);
        RenderSettings.ambientEquatorColor = new Color(0.21f, 0.12f, 0.09f);
        RenderSettings.ambientGroundColor = new Color(0.11f, 0.055f, 0.035f);

        CreateLighting();
        CreateDistantHorizonContext();
        CreateTerrain();
        CreateRocks();

        BaseMarker = CreateWaypoint("BASE", new Vector3(0f, 0f, 0f), new Color(0.18f, 0.80f, 0.92f));
        GeologyOutcropMarker = CreateWaypoint("GEOLOGY OUTCROP", new Vector3(38f, 0f, 9f), new Color(0.92f, 0.48f, 0.23f));
        FieldMarker = CreateWaypoint("FIELD MARKER", new Vector3(78f, 0f, -8f), new Color(0.80f, 0.26f, 0.90f));
        FinalMarker = CreateWaypoint("FINAL", new Vector3(116f, 0f, 4f), new Color(0.35f, 0.95f, 0.54f));
    }

    private void CreateLighting()
    {
        var sunObject = new GameObject("Low Sun");
        generatedObjects.Add(sunObject);
        var sun = sunObject.AddComponent<Light>();
        sun.type = LightType.Directional;
        sun.color = new Color(1f, 0.74f, 0.58f);
        sun.intensity = 2.15f;
        sun.shadows = LightShadows.Soft;
        sun.shadowStrength = 0.70f;
        sun.shadowBias = 0.04f;
        // Start in the mast camera's forward sector so the moving sun is immediately visible.
        var sunDirection = new Vector3(-0.12f, 0.38f, 0.92f).normalized;
        sun.transform.rotation = Quaternion.LookRotation(-sunDirection, Vector3.up);
        RenderSettings.sun = sun;

        var sunDisc = CreateCelestialBody("Sun disc", sunDirection * 520f, 34f, new Color(1f, 0.68f, 0.28f, 1f));
        var sunMotion = sunObject.AddComponent<MarsSunMotion>();
        sunMotion.Bind(sun, sunDisc.transform, sunDirection);

        var skybox = CreateMarsSkybox(sunDirection);
        if (skybox != null)
        {
            RenderSettings.skybox = skybox;
            DynamicGI.UpdateEnvironment();
        }

        CreateCelestialBody("Phobos", new Vector3(-280f, 285f, 700f), 10f, new Color(0.48f, 0.30f, 0.23f, 1f));
        CreateCelestialBody("Deimos", new Vector3(310f, 335f, 760f), 6f, new Color(0.68f, 0.53f, 0.42f, 1f));
    }

    private static Material CreateMarsSkyboxMaterial(Texture2D texture)
    {
        var shader = Shader.Find("Skybox/Panoramic") ?? Shader.Find("Skybox/Procedural");
        if (shader == null)
        {
            return null;
        }

        var material = new Material(shader) { name = "Mars Dust Skybox" };
        if (shader.name == "Skybox/Panoramic")
        {
            material.SetTexture("_MainTex", texture);
            material.SetColor("_Tint", Color.white);
            material.SetFloat("_Exposure", 0.85f);
            material.SetFloat("_Rotation", 0f);
        }
        else
        {
            material.SetColor("_SkyTint", new Color(0.10f, 0.028f, 0.018f, 1f));
            material.SetColor("_GroundColor", new Color(0.18f, 0.045f, 0.022f, 1f));
            material.SetFloat("_AtmosphereThickness", 0.68f);
            material.SetFloat("_SunSize", 0.018f);
            material.SetFloat("_SunSizeConvergence", 4.5f);
            material.SetFloat("_Exposure", 0.72f);
        }

        return material;
    }

    private static Material CreateCelestialMaterial(Color color)
    {
        // URP/Lit is already used by the foreground and is guaranteed to be in
        // the player build. Runtime-only Unlit shaders were stripped, which made
        // the moons and sun disappear from standalone builds.
        var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
        if (shader == null)
        {
            return null;
        }

        var material = new Material(shader) { name = "Mars Celestial Body Material" };
        material.SetColor("_BaseColor", color);
        material.SetColor("_Color", color);
        material.SetFloat("_Metallic", 0f);
        material.SetFloat("_Smoothness", 0f);
        material.EnableKeyword("_EMISSION");
        material.SetColor("_EmissionColor", color * 0.12f);
        return material;
    }

    private GameObject CreateCelestialBody(string name, Vector3 position, float diameter, Color color)
    {
        var body = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        body.name = name + " (SIM)";
        body.transform.position = position;
        body.transform.localScale = Vector3.one * diameter;
        var renderer = body.GetComponent<Renderer>();
        var material = CreateCelestialMaterial(color);
        if (name == "Sun disc")
        {
            material.SetColor("_EmissionColor", new Color(2.8f, 1.25f, 0.32f, 1f));
        }

        renderer.sharedMaterial = material;
        renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        renderer.receiveShadows = false;
        Destroy(body.GetComponent<Collider>());
        generatedObjects.Add(body);
        return body;
    }

    private static Material CreateMarsSkybox(Vector3 sunDirection)
    {
        const int width = 512;
        const int height = 256;
        var texture = new Texture2D(width, height, TextureFormat.RGBA32, false, true)
        {
            name = "Mars Dust Sky Gradient",
            wrapMode = TextureWrapMode.Repeat,
            filterMode = FilterMode.Bilinear
        };
        var pixels = new Color[width * height];
        for (var y = 0; y < height; y++)
        {
            var v = y / (float)(height - 1);
            var atmosphere = v < 0.5f
                ? Color.Lerp(new Color(0.06f, 0.035f, 0.028f), new Color(0.29f, 0.14f, 0.11f), v * 2f)
                : Color.Lerp(new Color(0.29f, 0.14f, 0.11f), new Color(0.10f, 0.075f, 0.095f), (v - 0.5f) * 2f);
            var horizonHaze = Mathf.Exp(-Mathf.Abs(v - 0.5f) * 28f);
            atmosphere = Color.Lerp(atmosphere, new Color(0.50f, 0.29f, 0.23f), horizonHaze * 0.34f);
            for (var x = 0; x < width; x++)
            {
                var u = x / (float)width;
                var dustBand = 0.96f + 0.04f * Mathf.Sin(u * Mathf.PI * 12f + v * 5f);
                var color = atmosphere * dustBand;
                pixels[y * width + x] = new Color(color.r, color.g, color.b, 1f);
            }
        }

        texture.SetPixels(pixels);
        texture.Apply(false, true);
        return CreateMarsSkyboxMaterial(texture);
    }

    private void CreateDistantHorizonContext()
    {
        // Keep horizon art opt-in until a reviewed local asset is selected.
        // The active drive view must remain foreground terrain plus the live skybox.
    }

    private void CreateTerrain()
    {
        const int resolution = 2049;
        var terrainData = new TerrainData
        {
            heightmapResolution = resolution,
            size = new Vector3(1024f, 36f, 768f),
            wavingGrassAmount = 0f,
            wavingGrassSpeed = 0f,
            wavingGrassStrength = 0f
        };

        var heights = LoadHeightmap(resolution);
        terrainData.SetHeights(0, 0, heights);
        var terrainObject = Terrain.CreateTerrainGameObject(terrainData);
        terrainObject.name = "Foreground Regolith Terrain";
        terrainObject.transform.position = new Vector3(-512f, 0f, -384f);
        Physics.SyncTransforms();
        generatedObjects.Add(terrainObject);
        foregroundTerrain = terrainObject.GetComponent<Terrain>();
        foregroundTerrain.heightmapPixelError = 4f;
        foregroundTerrain.basemapDistance = 260f;
        foregroundTerrain.drawHeightmap = false;
        RegolithMaterial = CreateRegolithMaterial();
        CreateTerrainSurface(terrainData, terrainObject.transform);
    }

    private static float[,] LoadHeightmap(int resolution)
    {
        var heightAsset = Resources.Load<TextAsset>(HeightmapResource);
        var expectedBytes = resolution * resolution * 2;
        if (heightAsset != null && heightAsset.bytes != null && heightAsset.bytes.Length >= expectedBytes)
        {
            var heights = new float[resolution, resolution];
            var bytes = heightAsset.bytes;
            for (var z = 0; z < resolution; z++)
            {
                for (var x = 0; x < resolution; x++)
                {
                    var offset = (z * resolution + x) * 2;
                    var sample = bytes[offset] | (bytes[offset + 1] << 8);
                    heights[x, z] = sample / 65535f;
                }
            }

            return heights;
        }

        Debug.LogWarning("LUSI Mars heightmap asset was not found; using the fallback terrain.");
        var fallback = new float[resolution, resolution];
        for (var x = 0; x < resolution; x++)
        {
            for (var z = 0; z < resolution; z++)
            {
                var u = x / (float)(resolution - 1);
                var v = z / (float)(resolution - 1);
                var broad = Mathf.PerlinNoise(u * 3.2f + 0.8f, v * 3.2f + 2.1f) * 0.035f;
                var fine = Mathf.PerlinNoise(u * 18.5f + 4.7f, v * 18.5f + 0.9f) * 0.012f;
                var dune = Mathf.Sin((u * 17f + v * 4.5f) * Mathf.PI) * 0.004f;
                fallback[x, z] = Mathf.Clamp01(0.055f + broad + fine + dune);
            }
        }

        return fallback;
    }

    private void CreateTerrainSurface(TerrainData terrainData, Transform terrainParent)
    {
        const int resolution = 513;
        var vertices = new Vector3[resolution * resolution];
        var uv = new Vector2[vertices.Length];
        var triangles = new int[(resolution - 1) * (resolution - 1) * 6];
        for (var z = 0; z < resolution; z++)
        {
            var v = z / (float)(resolution - 1);
            for (var x = 0; x < resolution; x++)
            {
                var u = x / (float)(resolution - 1);
                var index = z * resolution + x;
                vertices[index] = new Vector3(u * terrainData.size.x, terrainData.GetInterpolatedHeight(u, v), v * terrainData.size.z);
                uv[index] = new Vector2(u, v);
            }
        }

        var triangleIndex = 0;
        for (var z = 0; z < resolution - 1; z++)
        {
            for (var x = 0; x < resolution - 1; x++)
            {
                var bottomLeft = z * resolution + x;
                var bottomRight = bottomLeft + 1;
                var topLeft = bottomLeft + resolution;
                var topRight = topLeft + 1;
                triangles[triangleIndex++] = bottomLeft;
                triangles[triangleIndex++] = topLeft;
                triangles[triangleIndex++] = bottomRight;
                triangles[triangleIndex++] = bottomRight;
                triangles[triangleIndex++] = topLeft;
                triangles[triangleIndex++] = topRight;
            }
        }

        var mesh = new Mesh { name = "HiRISE Derived Mars Regolith Surface" };
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        var surface = new GameObject("Foreground Regolith Surface");
        surface.transform.SetParent(terrainParent, false);
        generatedObjects.Add(surface);
        var filter = surface.AddComponent<MeshFilter>();
        filter.sharedMesh = mesh;
        var renderer = surface.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = CreateSurfaceMaterial();
    }

    private Material CreateSurfaceMaterial()
    {
        var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
        var material = new Material(shader) { name = "Mars Foreground Surface Material" };
        var texture = Resources.Load<Texture2D>(AlbedoResource);
        if (texture != null)
        {
            texture.wrapMode = TextureWrapMode.Clamp;
            texture.filterMode = FilterMode.Trilinear;
            texture.anisoLevel = 4;
        }
        else
        {
            texture = CreateFallbackAlbedo();
        }

        material.SetColor("_BaseColor", Color.white);
        material.SetTexture("_BaseMap", texture);
        material.SetFloat("_Smoothness", 0.18f);
        var normal = Resources.Load<Texture2D>(NormalResource);
        if (normal != null)
        {
            material.EnableKeyword("_NORMALMAP");
            material.SetTexture("_BumpMap", normal);
            material.SetFloat("_BumpScale", 0.42f);
        }

        return material;
    }

    private Material CreateTerrainMaterial()
    {
        var shader = Shader.Find("Universal Render Pipeline/Terrain/Lit")
            ?? Shader.Find("Nature/Terrain/Standard")
            ?? Shader.Find("Universal Render Pipeline/Lit")
            ?? Shader.Find("Standard");
        var material = new Material(shader) { name = "Mars Native Terrain Material" };
        var texture = Resources.Load<Texture2D>(AlbedoResource);
        if (texture == null)
        {
            texture = CreateFallbackAlbedo();
        }

        texture.wrapMode = TextureWrapMode.Clamp;
        texture.filterMode = FilterMode.Trilinear;
        texture.anisoLevel = 4;
        material.SetColor("_BaseColor", Color.white);
        material.SetColor("_Color", Color.white);
        material.SetTexture("_BaseMap", texture);
        material.SetTexture("_MainTex", texture);
        material.SetFloat("_Smoothness", 0.16f);
        return material;
    }

    private static Texture2D CreateFallbackAlbedo()
    {
        var texture = new Texture2D(128, 128, TextureFormat.RGBA32, true, false)
        {
            name = "Fallback Mars Surface Albedo",
            wrapMode = TextureWrapMode.Repeat,
            filterMode = FilterMode.Bilinear,
            anisoLevel = 2
        };
        var pixels = new Color[128 * 128];
        for (var y = 0; y < 128; y++)
        {
            for (var x = 0; x < 128; x++)
            {
                var coarse = Mathf.PerlinNoise(x * 0.028f + 5.3f, y * 0.028f + 2.4f);
                var fine = Mathf.PerlinNoise(x * 0.17f + 1.7f, y * 0.17f + 6.1f);
                var value = 0.78f + coarse * 0.20f + fine * 0.10f;
                pixels[y * 128 + x] = new Color(0.36f * value, 0.135f * value, 0.075f * value, 1f);
            }
        }

        texture.SetPixels(pixels);
        texture.Apply(true, true);
        return texture;
    }

    private Material CreateRegolithMaterial()
    {
        var shader = RuntimeRegolithShader ?? Shader.Find("LUSI/MarsRegolith") ?? Shader.Find("Universal Render Pipeline/Lit");
        var material = new Material(shader) { name = "Mars Regolith Material" };
        material.SetColor("_BaseColor", new Color(0.37f, 0.15f, 0.095f, 1f));
        material.SetFloat("_Roughness", 0.92f);
        material.SetFloat("_NoiseScale", 0.35f);
        return material;
    }

    private void CreateRocks()
    {
        var rockMaterials = new[]
        {
            CreateRockMaterial("Basalt", new Color(0.48f, 0.18f, 0.095f, 1f), 0.48f),
            CreateRockMaterial("Oxidized", new Color(0.60f, 0.24f, 0.12f, 1f), 0.54f),
            CreateRockMaterial("Sunlit", new Color(0.72f, 0.32f, 0.16f, 1f), 0.62f)
        };

        CreateRock(rockMaterials[1], new Vector3(2.8f, 0.35f, 8f), new Vector3(1.5f, 1.1f, 1.3f), 32f, 0.78f, 0.15f);
        CreateRock(rockMaterials[0], new Vector3(-3.4f, 0.38f, 16f), new Vector3(1.8f, 1.4f, 1.6f), 118f, 0.86f, 0.42f);
        CreateRock(rockMaterials[2], new Vector3(5.5f, 0.32f, 24f), new Vector3(2.5f, 1.8f, 2f), 214f, 0.72f, 0.67f);
        CreateRock(rockMaterials[1], new Vector3(-6f, 0.34f, 31f), new Vector3(2.1f, 1.5f, 1.9f), 286f, 0.9f, 0.91f);

        var random = new System.Random(4317);
        for (var i = 0; i < 150; i++)
        {
            var x = -400f + (float)random.NextDouble() * 800f;
            var z = -330f + (float)random.NextDouble() * 660f;
            if (Mathf.Abs(z) < 3.5f && x < 125f)
            {
                z += z < 0f ? -5f : 5f;
            }

            var size = 0.16f + (float)random.NextDouble() * 2.1f;
            CreateRock(
                rockMaterials[random.Next(rockMaterials.Length)],
                new Vector3(x, 0.18f + size * 0.12f, z),
                new Vector3(size * (0.8f + (float)random.NextDouble() * 0.8f), size, size * (0.75f + (float)random.NextDouble() * 0.9f)),
                (float)random.NextDouble() * 360f,
                0.55f + (float)random.NextDouble() * 0.35f,
                (float)random.NextDouble());
        }
    }

    private Material CreateRockMaterial(string label, Color color, float noiseScale)
    {
        var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
        var material = new Material(shader);
        material.name = "Mars Rock Material - " + label;
        material.SetColor("_BaseColor", color);
        material.SetColor("_Color", color);
        material.SetFloat("_Metallic", 0f);
        material.SetFloat("_Smoothness", 0.10f);
        material.EnableKeyword("_EMISSION");
        material.SetColor("_EmissionColor", color * 0.14f);
        material.SetFloat("_NoiseScale", noiseScale);
        if (rockAlbedo == null)
        {
            rockAlbedo = CreateRockAlbedo();
        }

        material.SetTexture("_BaseMap", rockAlbedo);
        return material;
    }

    private static Texture2D CreateRockAlbedo()
    {
        const int resolution = 128;
        var texture = new Texture2D(resolution, resolution, TextureFormat.RGBA32, true, false)
        {
            name = "Procedural Mars Rock Albedo",
            wrapMode = TextureWrapMode.Repeat,
            filterMode = FilterMode.Trilinear,
            anisoLevel = 2
        };
        var pixels = new Color[resolution * resolution];
        for (var y = 0; y < resolution; y++)
        {
            for (var x = 0; x < resolution; x++)
            {
                var broad = Mathf.PerlinNoise(x * 0.028f + 11.2f, y * 0.028f + 3.6f);
                var fine = Mathf.PerlinNoise(x * 0.16f + 4.1f, y * 0.16f + 9.8f);
                var value = 0.76f + broad * 0.24f + fine * 0.12f;
                pixels[y * resolution + x] = new Color(0.70f * value, 0.28f * value, 0.13f * value, 1f);
            }
        }

        texture.SetPixels(pixels);
        texture.Apply(true, true);
        return texture;
    }

    private void CreateRock(Material rockMaterial, Vector3 position, Vector3 scale, float rotationY, float upperRingScale, float seed)
    {
        var rock = new GameObject("Regolith Rock");
        // The mesh has a broad, nearly-flat buried base.  Use the same interpolated
        // height as the visible terrain mesh so the contact point cannot drift from it.
        position.y = SampleTerrainSurfaceHeight(position) + scale.y * 0.44f;
        rock.transform.position = position;
        rock.transform.localScale = scale;
        rock.transform.rotation = Quaternion.Euler(0f, rotationY, 0f);
        generatedObjects.Add(rock);

        var filter = rock.AddComponent<MeshFilter>();
        filter.sharedMesh = CreateRockMesh(14, upperRingScale, seed);
        var renderer = rock.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = rockMaterial;
        var rockCollider = rock.AddComponent<MeshCollider>();
        rockCollider.sharedMesh = filter.sharedMesh;
        rockCollider.convex = true;
    }

    private static Mesh CreateRockMesh(int sides, float upperRingScale, float seed)
    {
        const int ringCount = 4;
        var vertices = new Vector3[sides * ringCount + 2];
        var triangles = new int[sides * 24];
        var profile = Mathf.Abs(Mathf.FloorToInt(seed * 9f)) % 3;
        var baseScale = profile == 1 ? 1.08f : profile == 2 ? 0.98f : 0.92f;
        var bodyScale = profile == 1 ? 1.16f : profile == 2 ? 0.96f : 1.02f;
        var shoulderScale = profile == 1 ? 1.14f : profile == 2 ? 0.84f : 0.98f;
        var capScale = profile == 1 ? 0.82f : profile == 2 ? 0.42f : 0.60f;
        var shoulderHeight = profile == 1 ? 0.20f : profile == 2 ? 0.40f : 0.31f;
        var capHeight = profile == 1 ? 0.34f : profile == 2 ? 0.56f : 0.47f;
        var peakHeight = profile == 1 ? 0.42f : profile == 2 ? 0.70f : 0.58f;
        var peakOffset = profile == 1 ? 0.16f : profile == 2 ? -0.12f : 0.05f;
        vertices[0] = new Vector3(0f, -0.48f, 0f);
        vertices[vertices.Length - 1] = new Vector3(
            peakOffset + 0.04f * Mathf.Sin(seed * 11f),
            peakHeight,
            peakOffset * 0.55f + 0.03f * Mathf.Cos(seed * 7f));
        for (var i = 0; i < sides; i++)
        {
            var angle = i * Mathf.PI * 2f / sides;
            var variation = 0.84f + 0.16f * Mathf.Sin(i * 4.1f + seed * 10f);
            var angleJitter = angle + 0.07f * Mathf.Sin(i * 2.3f + seed * 5f);
            var radiusJitter = 0.95f + 0.08f * Mathf.Sin(i * 1.71f + seed * 13f);
            var x = Mathf.Cos(angleJitter) * variation * radiusJitter;
            var z = Mathf.Sin(angleJitter) * variation * radiusJitter;
            // Bring the outer base down close to the center foot so the silhouette
            // meets the regolith instead of forming a dark open underside.
            vertices[1 + i] = new Vector3(x * baseScale, -0.44f + 0.018f * Mathf.Sin(i + seed), z * baseScale);
            vertices[1 + sides + i] = new Vector3(x * bodyScale, 0.15f + 0.06f * Mathf.Cos(i * 2.7f + seed), z * bodyScale);
            vertices[1 + sides * 2 + i] = new Vector3(x * shoulderScale * upperRingScale, shoulderHeight + 0.06f * Mathf.Sin(i * 1.9f + seed * 3f), z * shoulderScale * upperRingScale);
            vertices[1 + sides * 3 + i] = new Vector3(x * capScale * upperRingScale, capHeight + 0.035f * Mathf.Cos(i * 3.2f + seed), z * capScale * upperRingScale);
        }

        var t = 0;
        for (var i = 0; i < sides; i++)
        {
            var next = (i + 1) % sides;
            var lower = 1 + i;
            var nextLower = 1 + next;
            triangles[t++] = 0; triangles[t++] = nextLower; triangles[t++] = lower;
            for (var ring = 0; ring < ringCount - 1; ring++)
            {
                var current = 1 + ring * sides + i;
                var nextCurrent = 1 + ring * sides + next;
                var upper = 1 + (ring + 1) * sides + i;
                var nextUpper = 1 + (ring + 1) * sides + next;
                if ((i + ring) % 2 == 0)
                {
                    triangles[t++] = current; triangles[t++] = upper; triangles[t++] = nextCurrent;
                    triangles[t++] = upper; triangles[t++] = nextUpper; triangles[t++] = nextCurrent;
                }
                else
                {
                    triangles[t++] = current; triangles[t++] = nextUpper; triangles[t++] = nextCurrent;
                    triangles[t++] = current; triangles[t++] = upper; triangles[t++] = nextUpper;
                }
            }

            var cap = 1 + (ringCount - 1) * sides;
            triangles[t++] = cap + i; triangles[t++] = vertices.Length - 1; triangles[t++] = cap + next;
        }

        var mesh = new Mesh { name = "Hand-shaped basalt rock" };
        mesh.vertices = vertices;
        var uv = new Vector2[vertices.Length];
        for (var i = 0; i < vertices.Length; i++)
        {
            uv[i] = new Vector2(vertices[i].x * 1.15f + 0.5f, vertices[i].z * 1.15f + 0.5f);
        }

        mesh.uv = uv;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        return mesh;
    }

    private Transform CreateWaypoint(string label, Vector3 position, Color color)
    {
        position.y = SampleTerrainHeight(position) + 0.04f;
        var root = new GameObject(label);
        root.transform.position = position;
        generatedObjects.Add(root);

        var pad = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        pad.name = label + " landing pad";
        pad.transform.SetParent(root.transform);
        pad.transform.localPosition = Vector3.zero;
        pad.transform.localScale = new Vector3(3.2f, 0.08f, 3.2f);
        var padMaterial = CreateWaypointMaterial(new Color(color.r * 0.25f, color.g * 0.25f, color.b * 0.25f, 1f));
        pad.GetComponent<Renderer>().sharedMaterial = padMaterial;
        Destroy(pad.GetComponent<Collider>());

        var beacon = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        beacon.name = label + " beacon";
        beacon.transform.SetParent(root.transform);
        beacon.transform.localPosition = new Vector3(0f, 1.35f, 0f);
        beacon.transform.localScale = new Vector3(0.11f, 1.3f, 0.11f);
        var beaconMaterial = CreateWaypointMaterial(color);
        beacon.GetComponent<Renderer>().sharedMaterial = beaconMaterial;
        Destroy(beacon.GetComponent<Collider>());

        var lamp = new GameObject(label + " marker light");
        lamp.transform.SetParent(root.transform);
        lamp.transform.localPosition = new Vector3(0f, 2.75f, 0f);
        var point = lamp.AddComponent<Light>();
        point.type = LightType.Point;
        point.color = color;
        point.intensity = 1.8f;
        point.range = 8f;
        return root.transform;
    }

    private float SampleTerrainHeight(Vector3 worldPosition)
    {
        if (foregroundTerrain == null)
        {
            return 0.1f;
        }

        return SampleTerrainSurfaceHeight(worldPosition);
    }

    private float SampleTerrainSurfaceHeight(Vector3 worldPosition)
    {
        if (foregroundTerrain == null || foregroundTerrain.terrainData == null)
        {
            return 0.1f;
        }

        var local = worldPosition - foregroundTerrain.transform.position;
        var terrainSize = foregroundTerrain.terrainData.size;
        var u = Mathf.Clamp01(local.x / terrainSize.x);
        var v = Mathf.Clamp01(local.z / terrainSize.z);
        return foregroundTerrain.transform.position.y + foregroundTerrain.terrainData.GetInterpolatedHeight(u, v);
    }

    private Material CreateWaypointMaterial(Color color)
    {
        var shader = RuntimeRegolithShader ?? Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
        if (shader == null)
        {
            Debug.LogError("LUSI waypoint could not find a runtime-safe shader.");
            return null;
        }

        var material = new Material(shader);
        if (shader.name == "LUSI/MarsRegolith")
        {
            material.SetColor("_BaseColor", color);
        }
        else
        {
            material.color = color;
        }

        return material;
    }
}
