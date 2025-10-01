using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDCollision;
/// <summary>
/// Defines a Rectangle (or Axis-Aligned Bounding Box) for floating-point values (single-precision)
/// </summary>
// More technically, this implements directly against __m128 (System.Runtime.Intrinsics.Vector128), enabling the use of SIMD instructions.
// Layout is <xMin, yMin, -xMax, -yMax>
public struct RectangleF {
    internal float left; internal float top; internal float nRight; internal float nBottom;
    public float Left {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => left;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => Move(new Vector2(value - left, 0f));
    }
    public float Top {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => top;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => Move(new Vector2(0, value - top));
    }
    public float Right {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => -nRight;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => Move(new Vector2(value + nRight, 0f));
    }
    public float Bottom {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => -nBottom;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => Move(new Vector2(0, value + nBottom));
    }
    public float X {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => left;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => Move(new Vector2(value - left, 0f));
    }
    public float Y {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => top;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => Move(new Vector2(0, value - top));
    }
    public float Width {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => -(left + nRight);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set {
            if (value < 0) {
                nRight = -left;
                left = left + value;
            } else {
                nRight = -(left + value);
            }
        }
    }
    public float Height {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => -(top + nBottom);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set {
            if (value < 0) {
                nBottom = -top;
                top = top + value;
            } else {
                nBottom = -(top + value);
            }
        }
    }

    /// <summary>
    /// Gets the Rectangle as a 128bit vector, by reference. Values changed in the Vector will <i>not</i> translate over to the Rectangle without a cast back (VectorUtils.AsRectangleF)
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector128<float> AsVector128() => Unsafe.As<RectangleF, Vector128<float>>(ref this);

    public Vector2 Lower => AsVector128().AsVector2();
    public Vector2 Upper => Vector128.Shuffle(AsVector128(), Vector128.Create(2, 3, 0, 1)).AsVector2();

    public RectangleF(Vector2 position, float width, float height) {
        if (width < 0 || height < 0) {
            throw new ArgumentException("Width/Height cannot be negative!");
        }
        left = position.X;
        nRight = -(position.X + width);
        top = position.Y;
        nBottom = -(position.Y + height);
    }
    /// <summary>
    /// Sets the left value of the rectangle without moving the rectangle to match. Use rectangle.Left = value for setting Left normally.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Raw_SetLeft(float value) { left = value; }
    /// <summary>
    /// Sets the top value of the rectangle without moving the rectangle to match. Use rectangle.Top = value for setting Top normally.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Raw_SetTop(float value) { top = value; }
    /// <summary>
    /// Sets the right value of the rectangle without moving the rectangle to match. Use rectangle.Right = value for setting Right normally.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Raw_SetRight(float value) { nRight = -value; }
    /// <summary>
    /// Sets the bottom value of the rectangle without moving the rectangle to match. Use rectangle.Bottom = value for setting Bottom normally.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Raw_SetBottom(float value) { nBottom = -value; }

    /// <summary>
    /// Moves the rectangle some distance.
    /// </summary>
    /// <param name="change">The distance to move the rectangle.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Move(Vector2 change) => this = VectorUtils.AsRectangleF(this.AsVector128() + VectorUtils.InCollisionForm(change));

    // This order saves 1 instruction over Negate then Shuffle, on AVX. Lmao.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector128<float> Wind() => Vector128.Negate(Vector128.Shuffle(this.AsVector128(), Vector128.Create(2, 3, 0, 1)));

    /// <summary>
    /// Returns the union of two rectangles.<br/>
    /// min(a.Left, b.Left), min(a.Top, b.Top), max(a.Right, b.Right), max(a.Bottom, b.Bottom)
    /// </summary>
    // x86- (V)MINPS       ARM - VMINNM
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static RectangleF Union(RectangleF a, RectangleF b) => VectorUtils.AsRectangleF(Vector128.MinNative(a.AsVector128(), b.AsVector128()));
    /// <summary>
    /// Does <i><b>NOT</b></i> represent whether or not the two rectangles overlap. Returns the intersection of two rectangles.<br/>
    /// max(a.Left, b.Left), max(a.Top, b.Top), min(a.Right, b.Right), min(a.Bottom, b.Bottom)
    /// </summary>
    // x86- (V)MAXPS       ARM - VMAXNM
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static RectangleF Intersection(RectangleF a, RectangleF b) => VectorUtils.AsRectangleF(Vector128.MaxNative(a.AsVector128(), b.AsVector128()));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TestPointInRectangle(RectangleF rect, Vector2 point) => TestPointInRectangle(rect.AsVector128(), VectorUtils.InCollisionForm(point));
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TestPointInRectangle(Vector128<float> rect, Vector128<float> point) => !Vector128.LessThanAny(point, rect);

    /// <summary>
    /// Tests if two AABB overlap.
    /// </summary>
    /// <param name="a">Rectangle A</param>
    /// <param name="b">Rectangle B</param>
    /// <returns>Whether or not the two rectangles overlapped</returns>
    // the Rectangle "A" gets "wound" from (minX, minY, -maxX, -maxY) to (maxX, maxY, -minX, -minY). The calculation for overlaps can be simplified down to:
    //    a[0] >= b[0]      a[1] >= b[1]       a[2] >= b[2]        a[3] >= b[3]
    // (a.maxX >= b.minX, a.maxY >= b.minY, a.-minX >= b.-maxX, a.-minY >= b.-maxY)
    //                                      divide by -1
    // (a.maxX >= b.minX, a.maxY >= b.minY, a.minX  <= b.maxX , a.minY  <= b.maxY )
    // These are the four tests for standard overlap, and if all of them are true, then we have a collision!
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TestOverlap(RectangleF a, RectangleF b) => TestOverlap(a.AsVector128(), b.AsVector128());
    /// <summary>
    /// Tests if two AABB overlap, using vectorized instructions.<br/>Optimized for a <i>single</i> overlap, if you are testing one AABB against a set of AABBs, see documentation.
    /// </summary>
    // x86: vpermilps xmm0, [esp+0x14], 0x4e    ; load and shuffle a into 2 3 0 1 order (replaced with movups shufps 78 without avx)
    // AVX  vxorps xmm0, xmm0, ALLBITSSET       ; negation of a
    //      vcmpltps xmm0, xmm0, [esp + 4]      ; for each float: xmm0[i] = a[i] < b[i] ? ALLBITSSET : 0 (a[i] >= b[i] ? 0 : ALLBITSSET)
    //      vptest xmm0, xmm0                   ; if xmm0[1:4] & xmm0[1:4] = all zeros, set ZF to 1 (if a[i] >= b[i], set ZF to 1)
    //      sete al                             ; set byte [al] if ZF == 1
    //      movzx eax, al                       ; moves [al] (1byte) to eax (4bytes), zeroing the extra bytes
    //      ret 0x20
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TestOverlap(Vector128<float> rectA, Vector128<float> rectB) {
        rectA = Vector128.Negate(Vector128.Shuffle(rectA, Vector128.Create(2, 3, 0, 1)));
        return !Vector128.LessThanAny(rectA, rectB);
    }
    /// <summary>
    /// Tests if two AABB overlap with an Edge Condition, using vectorized instructions.<br/>Optimized for a <i>single</i> overlap, if you are testing one AABB against a set of AABBs, see documentation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TestOverlapWithRule(RectangleF rectA, RectangleF rectB, EdgeCollisionRule rule) {
        var a = Vector128.Negate(Vector128.Shuffle(rectA.AsVector128(), Vector128.Create(2, 3, 0, 1))); // Inlined
        var b = rectB.AsVector128(); // No operation. Just makes the C# compiler treat it as the right data\
        // Cheeky microoptimizations in here:
        // mask = (rule & <1,2,4,8> != 0), ~mask = (rule & <1,2,4,8> == 0)
        var nMask = Vector128.Equals(Vector128.Create((int)rule) & Vector128.Create(1, 2, 4, 8), Vector128<int>.Zero).AsSingle();
        // a > b | (a==b & mask) => ~( a<=b & (a!=b | ~mask) )
        return !Vector128.EqualsAny(Vector128.LessThanOrEqual(a, b), Vector128.Equals(a, b) | nMask);
    }
    
    // If you're here for one against a set of AABBs,
    // Wind A yourself and test overlaps with rectangle B by converting to AsVector128 and then running TestOverlap (which will get inlined).
    // Only one rectangle needs to "change" for the function to work,
    // so changing the "one" that the set goes against means you dont need to change the set (i.e. more instructions)
    /// <summary>
    /// Example iteration over a group. Removes values from the list that do not collide with rectA.
    /// </summary>
    /// <param name="rectA"></param>
    /// <param name="group"></param>
    /// <remarks>Your code will be <i>significantly</i> faster if you write this yourself, and you know what you're doing.<br/>See the source code for more info.</remarks>
    public static bool TestRectangleAgainstGroup(RectangleF rectA, ref List<RectangleF> group) {
        bool ret = false;
        Vector128<float> a = Vector128.Negate(Vector128.Shuffle(rectA.AsVector128(), Vector128.Create(2, 3, 0, 1)));

        for(int i = group.Count - 1; i > -1; i--) {
            if (!Vector128.LessThanAny(a, group[i].AsVector128())) {
                ret = true;
            } else {
                group.RemoveAt(i);
            }
        }
        return ret;
    }
}
