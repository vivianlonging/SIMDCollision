using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Text;
using System.Threading.Tasks;
using static System.Collections.Specialized.BitVector32;

namespace SIMDCollision;
public static partial class Collisions {
    #region Point
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CheckPoint(Vector2 a, Vector2 b) => a == b;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CheckPoint(Vector2 a, Vector2 b, float discrepancy) => Vector128.LessThanOrEqualAll(Vector128.Abs(b.AsVector128() - a.AsVector128()), Vector128.Create(discrepancy));
    #endregion

    #region Line
    internal static bool onSegment(Vector2 p, Vector2 q, Vector2 r) {
        Vector128<float> _p = VectorUtils.InCollisionForm(p);
        Vector128<float> _q = VectorUtils.InCollisionForm(q);
        Vector128<float> _r = VectorUtils.InCollisionForm(r);
        return Vector128.LessThanOrEqualAll(_q, Vector128.Max(_p, _r));
    }
    public static bool LineCheck(Vector2 aFrom, Vector2 aTo, Vector2 bFrom, Vector2 bTo) {
        // orientation is: Math.Sign(Vector2.PerpDot(b - a, c - b));
        int o1 = Math.Sign(VectorUtils.PerpDot(bFrom - aFrom, aTo - bFrom));
        int o2 = Math.Sign(VectorUtils.PerpDot(bFrom - aFrom, bTo - bFrom));
        int o3 = Math.Sign(VectorUtils.PerpDot(bTo - aTo, aFrom - bTo));
        int o4 = Math.Sign(VectorUtils.PerpDot(bTo - aTo, bFrom - bTo));
        if (o1 != o2 && o3 != o4) return true;
        else if (o1 == 0 && onSegment(aFrom, aTo, bFrom)) return true; // aFrom, bFrom, and aTo are collinear and aTo lies on segment aFrom-bFrom
        else if (o2 == 0 && onSegment(aFrom, bTo, bFrom)) return true; // aFrom, bFrom, and bTo are collinear and bTo lies on segment aFrom-bFrom
        else if (o3 == 0 && onSegment(aTo, aFrom, bTo)) return true; // aTo, bTo, and aFrom are collinear and aFrom lies on segment aTo-bTo
        else if (o4 == 0 && onSegment(aTo, bFrom, bTo)) return true; //aTo, bTo, and bFrom are collinear and bFrom lies on segment aTo-bTo
        return false;
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)] public static bool LineToCircle(Vector2 start, Vector2 end, Circle circle) => CircleToLine(circle, start, end);
    [MethodImpl(MethodImplOptions.AggressiveInlining)] public static bool LineToRect(Vector2 start, Vector2 end, RectangleF rect, EdgeCollisionRule rule) => RectToLine(rect, start, end, rule);


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2 ClosestPointOnLine(Vector2 start, Vector2 end, Vector2 closestTo) {
        Vector2 v = end - start;
        float t = Vector2.Dot(closestTo - start, v) / Vector2.Dot(v, v);
        t = float.Clamp(t, 0f, 1f);
        return start + v * t;
    }
    #endregion

    #region Circle
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CheckCircle(Circle c1, Circle c2) {
        float r = c1.Radius + c2.Radius;
        return Vector2.DistanceSquared(c1.Position, c2.Position) < r * r;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CircleToLine(Circle c, Vector2 start, Vector2 end) => Vector2.DistanceSquared(c.Position, ClosestPointOnLine(start, end, c.Position)) < c.RadiusSq;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CircleToPoint(Circle c, Vector2 point) => Vector2.DistanceSquared(c.Position, point) < c.RadiusSq;

    [MethodImpl(MethodImplOptions.AggressiveInlining)] public static bool CircleToRect(Circle circle, RectangleF rect) => RectToCircle(rect, circle);
    #endregion

    #region Rect
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CheckRect(RectangleF rectA, RectangleF rectB) => RectangleF.TestOverlap(rectA, rectB);
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CheckRect(RectangleF rectA, RectangleF rectB, [ConstantExpected] EdgeCollisionRule rule) => RectangleF.TestOverlapWithRule(rectA, rectB, rule);
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool RectToCircle(RectangleF rect, Circle circle) {
        Vector2 point = circle.Position;
        Vector2 test = Vector2.Clamp(point, rect.Lower, rect.Upper);
        return Vector2.DistanceSquared(point, test) < circle.RadiusSq;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool RectToLine(RectangleF rect, Vector2 start, Vector2 end, EdgeCollisionRule rule) => RectToLine_Internal(rect.AsVector128(), VectorUtils.InCollisionForm(start), VectorUtils.InCollisionForm(end), rule);
    // These are based on the Cohen-Sutherland algorithm, however, instead of using Outcodes we just use the Vector128<float> representation since we already have it, this vectorizes our answer.
    internal static bool RectToLine_Internal(Vector128<float> rect, Vector128<float> start, Vector128<float> end, EdgeCollisionRule rule) {
        // Order of outcode is <left, top, right, bottom> = 0b0001,0b0010,0b0100,0b1000
        // Scalar form:
        // s.Left   = start.X < rect.Left     || (rule & Rule.Left   != 0 ? start.X == rect.Left   : false)
        // s.Top    = start.Y < rect.Top      || (rule & Rule.Top    != 0 ? start.Y == rect.Top    : false)
        // s.Right  = -start.X < -rect.Right  || (rule & Rule.Right  != 0 ? start.X == rect.Right  : false)
        // s.Bottom = -start.Y < -rect.Bottom || (rule & Rule.Bottom != 0 ? start.Y == rect.Bottom : false)
        // We do this with:                    where ABCD are i = 0,1,2,3
        // s[i] = start[i] < rect[i] || ((rule & 0bDCBA != 0 ? 0b111... : 0b000...) & start[i] == rect[i])
        //                           if rule & Rule.Left/Top/Right/Bottom == 0, this right side is all zeroes, aka "false", a || false = a
        Vector128<int> ruleCover = Vector128.Create((int)rule) & Vector128.Create(1, 2, 4, 8);
        Vector128<int> ruleMask = ~Vector128.Equals(ruleCover, Vector128<int>.Zero);
        // order is -x, -y, +x, +y or "left", "top", "right", "bottom" in worldspace - we order the checks according to Cohen-Sutherland in the if statement
        Vector128<int> outS = Vector128.LessThan(start, rect).AsInt32() | (Vector128.Equals(start.AsInt32(), rect.AsInt32()) & ruleMask);
        Vector128<int> outE = Vector128.LessThan(end, rect).AsInt32() | (Vector128.Equals(end.AsInt32(), rect.AsInt32()) & ruleMask);
        while (true) {
            if (Vector128.EqualsAll(Vector128.BitwiseOr(outS, outE), Vector128<int>.Zero)) return true;
            else if (!Vector128.EqualsAll(Vector128.BitwiseAnd(outS, outE), Vector128<int>.Zero)) return false;
            else {
                Vector128<float> diff = end - start;
                Vector2 v = Vector2.Zero;
                // equivalent to movemask, but is actually crossplatform.
                int s = Vector128.Dot(outS & ruleCover, Vector128.Create(1)); // Dot Product against <1,1,1,1>, we're making the assumption that most hardware uses the appropriate intrinsic for this.
                int e = Vector128.Dot(outE & ruleCover, Vector128.Create(1));
                bool a = e > s;
                int b = a ? e : s;
                // I added these elses based on the Wikipedia article doing so. Might be worth keeping an eye on.
                if ((b & 8) != 0) {
                    v.X = start[0] + diff[0] * (rect[1] - start[1]) / diff[1];
                    v.Y = rect[1];
                } else if ((b & 4) != 0) {
                    v.Y = start[1] + diff[1] * (rect[0] + start[0]) / diff[0];
                    v.X = rect[0];
                } else if ((b & 2) != 0) {
                    v.X = start[0] + diff[0] * (start[1] + rect[3]) / diff[3];
                    v.Y = -rect[3];
                } else if ((b & 1) != 0) {
                    v.Y = start[1] + diff[1] * (start[0] + rect[2]) / diff[2];
                    v.X = -rect[2];
                }
                if (a) {
                    end = VectorUtils.InCollisionForm(v);
                    outE = Vector128.LessThan(end, rect).AsInt32() | (Vector128.Equals(end.AsInt32(), rect.AsInt32()) & ruleMask);
                } else {
                    start = VectorUtils.InCollisionForm(v);
                    outS = Vector128.LessThan(start, rect).AsInt32() | (Vector128.Equals(start.AsInt32(), rect.AsInt32()) & ruleMask);
                }
            }
        }
    }
    #endregion

}