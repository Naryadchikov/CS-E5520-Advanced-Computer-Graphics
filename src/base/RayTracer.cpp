#define _CRT_SECURE_NO_WARNINGS

#include "base/Defs.hpp"
#include "base/Math.hpp"
#include "RayTracer.hpp"
#include <stdio.h>
#include "rtIntersect.inl"
#include <fstream>

#include "rtlib.hpp"


// Helper function for hashing scene data for caching BVHs
extern "C" void MD5Buffer(void* buffer, size_t bufLen, unsigned int* pDigest);


namespace FW
{
    Vec2f getTexelCoords(Vec2f uv, const Vec2i size)
    {
        // YOUR CODE HERE (R3):
        // Get texel indices of texel nearest to the uv vector. Used in texturing.
        // UV coordinates range from negative to positive infinity. First map them
        // to a range between 0 and 1 in order to support tiling textures, then
        // scale the coordinates by image resolution and find the nearest pixel.
        float x, y, i;

        x = std::modf(uv.x, &i);
        y = std::modf(uv.y, &i);

		if (x < 0.f)
		{
			x += 1.f;
		}
		
		if (y < 0.f)
		{
			y += 1.f;
		}
        
		int ix = x * size.x;
		int iy = y * size.y;

		ix = FW::clamp(ix, 0, size.x);
		iy = FW::clamp(iy, 0, size.y);

        return Vec2f(ix, iy);
    }

    Mat3f formBasis(const Vec3f& n)
    {
        Vec3f q = n;
        int idOfMin = 0;

        for (int i = 1; i < 3; ++i)
        {
            if (FW::abs(n[idOfMin]) > FW::abs(n[i]))
            {
                idOfMin = i;
            }
        }

        q[idOfMin] = 1.f;

        Vec3f t = (FW::cross(q, n)).normalized();
        Vec3f b = (FW::cross(n, t)).normalized();

        Mat3f R;

        R.setCol(0, t);
        R.setCol(1, b);
        R.setCol(2, n);

        return R;
    }


    String RayTracer::computeMD5(const std::vector<Vec3f>& vertices)
    {
        unsigned char digest[16];
        MD5Buffer((void*)&vertices[0], sizeof(Vec3f) * vertices.size(), (unsigned int*)digest);

        // turn into string
        char ad[33];
        for (int i = 0; i < 16; ++i)
            ::sprintf(ad + i * 2, "%02x", digest[i]);
        ad[32] = 0;

        return FW::String(ad);
    }


    // --------------------------------------------------------------------------


    RayTracer::RayTracer()
    {
    }

    RayTracer::~RayTracer()
    {
    }


    void RayTracer::loadHierarchy(const char* filename, std::vector<RTTriangle>& triangles)
    {
        std::ifstream ifs(filename, std::ios::binary);
        m_bvh = Bvh(ifs);

        m_triangles = &triangles;
    }

    void RayTracer::saveHierarchy(const char* filename, const std::vector<RTTriangle>& triangles)
    {
        (void)triangles; // Not used.

        std::ofstream ofs(filename, std::ios::binary);
        m_bvh.save(ofs);
    }

    void RayTracer::constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode)
    {
        // YOUR CODE HERE (R1):
        // This is where you should construct your BVH.
        m_bvh = Bvh(triangles, splitMode);

        m_triangles = &triangles;
    }


    RaycastResult RayTracer::raycast(const Vec3f& orig, const Vec3f& dir) const
    {
        ++m_rayCount;

        // YOUR CODE HERE (R1):
        // This is where you traverse the tree you built! It's probably easiest
        // to introduce another function above that does the actual traversal, and
        // use this function only to begin the recursion by calling the traversal
        // function with the given ray and your root node. You can also use this
        // function to do one-off things per ray like finding the elementwise
        // reciprocal of the ray direction.
        float tMin = 1.f;
		Vec3f iDir = 1.f / dir;

        return intersectNode(orig, dir, iDir, m_bvh.root(), tMin);
    }

    RaycastResult RayTracer::intersectNode(const Vec3f& orig, const Vec3f& dir, const Vec3f& iDir, const BvhNode& node, float& tMin) const
    {
        // do it for children nodes to reduce recursion (if triangle from one child is in front of next BB -> skip)
		if (!isIntersectedWithBB(orig, iDir, node.bb, tMin))
        {
            return RaycastResult();
        }

        if (!node.left && !node.right)
        {
            return intersectTriangles(orig, dir, node.startPrim, node.endPrim, tMin);
        }

        RaycastResult leftResult = intersectNode(orig, dir, iDir, *node.left, tMin);
        RaycastResult rightResult = intersectNode(orig, dir, iDir, *node.right, tMin);

        if (!leftResult.tri && !rightResult.tri)
        {
            return RaycastResult();
        }

        return leftResult.t < rightResult.t ? leftResult : rightResult;
    }

    bool RayTracer::isIntersectedWithBB(const Vec3f& orig, const Vec3f& iDir, const AABB& bb, float& tMin) const
    {
        Vec3f t1 = (bb.min - orig) * iDir;
        Vec3f t2 = (bb.max - orig) * iDir;

        float start = FW::min(t1, t2).max();
        float end = FW::max(t1, t2).min();

		if (start > end || end < 0 || start > tMin)
        {
            return false;
        }

        return true;
    }

    RaycastResult RayTracer::intersectTriangles(const Vec3f& orig, const Vec3f& dir, const size_t startPrim,
                                                const size_t endPrim, float& tMin) const
    {
        float tmin = 1.0f, umin = 0.0f, vmin = 0.0f;
        int imin = -1;

        for (size_t i = startPrim; i <= endPrim; ++i)
        {
            float t, u, v;

            if ((*m_triangles)[m_bvh.getIndex(i)].intersect_woop(orig, dir, t, u, v))
            {
                if (t > 0.0f && t < tmin)
                {
                    imin = i;
                    tmin = t;
                    umin = u;
                    vmin = v;
                }
            }
        }

		if (imin != -1)
		{
			tMin = tmin;
			return RaycastResult(&(*m_triangles)[m_bvh.getIndex(imin)], tmin, umin, vmin, orig + tmin * dir, orig, dir);
		}

        return RaycastResult();
    }
} // namespace FW
