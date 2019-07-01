#ifndef SR_CONVEX_HULL
#define SR_CONVEX_HULL

// From Morgan + a bugfix from us

void swap(inout float3 a, inout float3 b) 
{
    // Note that there are clever ways of doing this without a
    // temporary, but they risk precision issues
    float3 temp = a;
    a = b;
    b = temp;
}

void swap(inout float2 a, inout float2 b)
{
    // Note that there are clever ways of doing this without a
    // temporary, but they risk precision issues
    float2 temp = a;
    a = b;
    b = temp;
}

/** 
Returns true if going from p1 to p2  to p3 turns to the right at p2, where
e12 = p2 - p1, e13 = p3 - p1.

If we're walking in CCW order, then turning left or going straight 
indicates that p2 is not on the convex hull.
*/
bool isRightTurn(const in float2 e12, const in float2 e13)
{
    // Note that this expression is the z component of the cross product,
    // which is also a determininant.  If it is positive, then we've made a left turn (i.e., we wrapped CCW).
    // If it is not positive, then we made a right turn (i.e., wrapped CW). Nothing needs to be normalized
    // since we only care about the sign.
    return (e12.x * e13.y - e12.y * e13.x) < 0.0f;
}


bool yxLessThan(const in float2 a, const in float2 b) 
{
    return ((a.y < b.y) || (a.y == b.y) && (a.x < b.x));
}

/** Store the CCW sort key relative to v0 in v.z */
void computeSortKey(const in float3 v0, inout float3 v)
{
    float2 delta = v.xy - v0.xy;
    // This should compile to an efficient rsqrt
    v.z = delta.x / sqrt(dot(delta, delta));
}

/** Sorts two elements by their z field */
void zsort(inout float3 a, inout float3 b)
{
    if (a.z > b.z)
    {
        swap(a, b);
    }
}

/** Sorts two elements by their xy fields */
void xysort(inout float2 a, inout float2 b)
{
    if (yxLessThan(b, a))
    { 
        swap(a, b);
    }
}

/** Computes the convex hull of points v0...v5 and emits it as a single triangle strip. */
void findConvexHull(in float3 v0, in float3 v1, in float3 v2, in float3 v3, in float3 v4, in float3 v5, inout float2 vecs[9], out int cnt) 
{
    // Swap the lowest y vertex to position 0.
    // In the case of ties, resolve to the lower x
    // (the latter is specified by Graham, but does it matter?)
    // 5 comparisons
    xysort(v0.xy, v1.xy);
    xysort(v0.xy, v2.xy);
    xysort(v0.xy, v3.xy);
    xysort(v0.xy, v4.xy);
    xysort(v0.xy, v5.xy);

    // Compute sort keys relative to v0.  Store them in the z component.
    computeSortKey(v0, v1);
    computeSortKey(v0, v2);
    computeSortKey(v0, v3);
    computeSortKey(v0, v4);
    computeSortKey(v0, v5);

    // Sort by keys in increasing (CCW) order-- 10 comparisons in a sorting network.
    zsort(v1, v2);    zsort(v1, v3);    zsort(v1, v4);    zsort(v1, v5);
    zsort(v2, v3);    zsort(v2, v4);    zsort(v2, v5);    
    zsort(v3, v4);    zsort(v3, v5);
    zsort(v4, v5);

    // The first two vertices and the last must be on the convex hull.
    // Use v5 as the apex of the fan; we'll keep coming back to it.
    vecs[0] = v0.xy;
    vecs[1] = v5.xy;
    vecs[2] = v1.xy;
    cnt = 3;
    // Last known "good" vertex on the CH, constantly updated
    float2 G = v1.xy;

    // Now we must determine which of points [2,3,4], if any, are on the hull.

    // Vertex 2: check corners 123, 124, 125
    float2 toNext = v2.xy - G;
    if (isRightTurn(toNext, v3.xy - G) &&
        isRightTurn(toNext, v4.xy - G) &&
        isRightTurn(toNext, v5.xy - G)) 
    {
        vecs[cnt  ] = v5.xy;
        vecs[cnt+1] = v2.xy;
        cnt += 2;
        G = v2.xy;
    }

    // Vertex 3: check corners G34, G35
    toNext = v3.xy - G;
    if (isRightTurn(toNext, v4.xy - G) &&
        isRightTurn(toNext, v5.xy - G))
    {
        vecs[cnt  ] = v5.xy;
        vecs[cnt+1] = v3.xy;
        cnt += 2;
        G = v3.xy;
    }

    // Vertex 4: check corner G45
    if (isRightTurn(v4.xy - G, v5.xy - G)) 
    {
        vecs[cnt  ] = v5.xy;
        vecs[cnt+1] = v4.xy;
        cnt += 2;
        G = v4.xy;
    }
}

#define inmax  12
#define outmax (3 + (inmax - 3) * 2)
void findConvexHullExt(in float3 vs[inmax], inout float2 vecs[outmax], out int cnt) 
{
    // Swap the lowest y vertex to position 0.
    // In the case of ties, resolve to the lower x
    // (the latter is specified by Graham, but does it matter?)
    // 5 comparisons
    int i, j;
    for( i = 1; i < inmax; i++ )
    {
        xysort( vs[0].xy, vs[i].xy );
    }

    // Compute sort keys relative to v0.  Store them in the z component.
    for( i = 1; i < inmax; i++ )
    {
        computeSortKey( vs[0], vs[i] );
    }

    for( i = 1; i < inmax-1; i++ )
        for( j = i+1; j < inmax; j++)
        {
            zsort(vs[i], vs[j] );
        }

    // The first two vertices and the last must be on the convex hull.
    // Use v5 as the apex of the fan; we'll keep coming back to it.
    vecs[0] = vs[0].xy;
    vecs[1] = vs[inmax-1].xy;
    vecs[2] = vs[1].xy;
    cnt = 3;
    // Last known "good" vertex on the CH, constantly updated
    float2 G = vs[1].xy;

    // Now we must determine which of points [2,3,4], if any, are on the hull.

    for( i = 2; i < inmax-1; i++ )
    {
        float2 toNext = vs[i].xy - G;
        bool isRight = true;
        for( j = i+1; j < inmax; j++)
        {
            isRight = isRight && isRightTurn( toNext, vs[j].xy - G );
        }
        if( isRight )
        {
            vecs[cnt  ] = vs[inmax-1].xy;
            vecs[cnt+1] = vs[i].xy;
            cnt += 2;
            G = vs[i].xy;
        }
    }
}

#endif // #define SR_CONVEX_HULL
