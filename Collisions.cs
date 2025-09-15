using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.Wasm;
using System.Runtime.Intrinsics.X86;
using System.Text;
using System.Threading.Tasks;
using static System.Collections.Specialized.BitVector32;

namespace SIMDCollision;
public static partial class Collisions {
    #region Point
    // Trying to optimize these results in worse code usually
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CheckPoint(Vector2 a, Vector2 b) => a == b;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CheckPoint(Vector2 a, Vector2 b, float discrepancy) => Vector128.LessThanOrEqualAll(Vector128.Abs(b.AsVector128() - a.AsVector128()), Vector128.Create(discrepancy));
    #endregion

    #region Line
    internal static bool onSegment(Vector128<float> p, Vector128<float> q, Vector128<float> r) {
        return Vector128.LessThanOrEqualAll(q, Vector128.MaxNative(p, r));
    }
    // 0bS EEEEEEEEFFFFFFFFFFFFFFFFFFFFFFF
// becomes
    // 0bS  any !=0 => 0x00...01
    // This breaks on NaN propagation as well as -0, however since we bitcast back to floats for 0 cost later, this resolves itself.
    internal static int fSign(float f) {
        int i = BitConverter.SingleToInt32Bits(f);
        return i & (1 << 31) // sign bit preserved
            | Convert.ToInt32(Convert.ToBoolean(i & 2147483647)); // !!(i & 0x7fffffff)
    }

    public static bool LineCheck(Vector2 aFrom, Vector2 aTo, Vector2 bFrom, Vector2 bTo) {
        Vector128<float> af = aFrom.AsVector128Unsafe(), at = aTo.AsVector128Unsafe(), bf = bFrom.AsVector128Unsafe(), bt = bTo.AsVector128Unsafe();
        Vector128<float> dF = bf - af; Vector128<float> dT = bt - at;
        // variation of sign that breaks NaN propagation in exchange for a lot of speedup
        dF = Vector128.Create(VectorUtils.PerpDot(dF, at - bf),
                              VectorUtils.PerpDot(dF, bt - bf),
                              VectorUtils.PerpDot(dT, af - bt),
                              VectorUtils.PerpDot(dT, bf - bt));
        dT = Vector128.Create(2147483647).AsSingle();
        dF = ((dF.AsInt32() & Vector128.Create(1 << 31)) 
             | (Vector128.Equals(dF.AsInt32() & dT.AsInt32(), Vector128<int>.Zero) & dT.AsInt32())
            ).AsSingle();

        if (dF[0] != dF[1] && dF[2] != dF[3]) return true;
        else
            return (dF[0] == 0 && onSegment(af, at, bf)) || // aFrom, bFrom, and aTo are collinear and aTo lies on segment aFrom-bFrom
                   (dF[1] == 0 && onSegment(af, bt, bf)) || // aFrom, bFrom, and bTo are collinear and bTo lies on segment aFrom-bFrom
                   (dF[2] == 0 && onSegment(at, af, bt)) || // aTo, bTo, and aFrom are collinear and aFrom lies on segment aTo-bTo
                   (dF[3] == 0 && onSegment(at, bf, bt));   //aTo, bTo, and bFrom are collinear and bFrom lies on segment aTo-bTo
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)] public static bool LineToCircle(Vector2 start, Vector2 end, Circle circle) => CircleToLine(circle, start, end);
    [MethodImpl(MethodImplOptions.AggressiveInlining)] public static bool LineToRect(Vector2 start, Vector2 end, RectangleF rect, EdgeCollisionRule rule) => RectToLine(rect, start, end, rule);


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2 ClosestPointOnLine(Vector2 start, Vector2 end, Vector2 closestTo) {
        if (Sse41.IsSupported) {
            // This saves a few instructions
            Vector128<float> s = start.AsVector128Unsafe(), e = end.AsVector128Unsafe(), c = closestTo.AsVector128Unsafe(),
                             v = e - s;
            // DotProduct with control byte only calcs dp on bits set
            e = Vector128.ClampNative(
                Sse41.DotProduct(c - s, v, 0b00110011) / Sse41.DotProduct(v, v, 0b00110011),
                Vector128<float>.Zero, Vector128<float>.One);
            return (s + v * e).AsVector2();
        } else { // No other instruction set has a dotproduct/horizontal add with a bitmask.
            Vector128<float> s = start.AsVector128(), e = end.AsVector128(), c = closestTo.AsVector128();
            Vector128<float> v = e - s;
            float t = Vector128.Dot(c - s, v) / Vector128.Dot(v, v);
            t = float.Clamp(t, 0f, 1f);
            return (s + v * t).AsVector2();
        }
    }
    #endregion

    #region Circle
    public static bool CheckCircle(Circle circle1, Circle circle2) {
        Vector128<float> c1 = circle1.AsVector128(), c2 = circle2.AsVector128();
        float r = c1[2] + c2[2];
        Vector128<float> c3 = (c2 - c1) * Vector128.Create(1f,1f,0f,0f);
        return Vector128.Dot(c3,c3) < r * r;
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CircleToLine(Circle circle, Vector2 start, Vector2 end) => Vector2.DistanceSquared(circle.Position, ClosestPointOnLine(start, end, circle.Position)) < circle.RadiusSq;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CircleToPoint(Circle circle, Vector2 point) => Vector2.DistanceSquared(circle.Position, point) < circle.RadiusSq;

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
        Vector2 test = Vector2.ClampNative(point, rect.Lower, rect.Upper);
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
        // These two lines above are equivalent to the RuleToMask function, but since we need the cover we have to write it manually here.
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