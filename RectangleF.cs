using System;
using System.Collections.Generic;
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
public struct RectangleF {
    private float left; private float top; private float nRight; private float nBottom;
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
    /// Gets the Rectangle as a 128bit vector, by reference. Values changed in the Vector will *not* translate over to the Rectangle without a cast back (VectorUtils.AsRectangleF)
    /// </summary>
    /// <returns></returns>
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


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector128<float> Wind() => Vector128.Shuffle(Vector128.Negate(this.AsVector128()), Vector128.Create(2, 3, 0, 1));

    /// <summary>
    /// Returns the union of two rectangles.<br/>
    /// min(a.Left, b.Left), min(a.Top, b.Top), max(a.Right, b.Right), max(a.Bottom, b.Bottom)
    /// </summary>
    // x86- (V)MINPS       ARM - VMINNM
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static RectangleF Union(RectangleF a, RectangleF b) => VectorUtils.AsRectangleF(VectorUtils.Min(a.AsVector128(), b.AsVector128()));
    /// <summary>
    /// Does <i><b>NOT</b></i> represent whether or not the two rectangles overlap. Returns the intersection of two rectangles.<br/>
    /// max(a.Left, b.Left), max(a.Top, b.Top), min(a.Right, b.Right), min(a.Bottom, b.Bottom)
    /// </summary>
    // x86- (V)MAXPS       ARM - VMAXNM
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static RectangleF Intersection(RectangleF a, RectangleF b) => VectorUtils.AsRectangleF(VectorUtils.Max(a.AsVector128(), b.AsVector128()));

    /// <summary>
    /// Tests if two AABB overlap.
    /// </summary>
    /// <param name="a">Rectangle A</param>
    /// <param name="b">Rectangle B</param>
    /// <returns>Whether or not the two rectangles overlapped</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TestOverlap(RectangleF a, RectangleF b) => TestOverlap(a.AsVector128(), b.AsVector128());
    /// <summary>
    /// Tests if two AABB overlap, using vectorized instructions.<br/>Optimized for a *single* overlap, if you are testing one AABB against a set of AABBs, see documentation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TestOverlap(Vector128<float> rectA, Vector128<float> rectB) {
        rectA = Vector128.Shuffle(-rectA, Vector128.Create(2, 3, 0, 1));
        return Vector128.GreaterThanOrEqualAll(rectA, rectB);
    }
    /// <summary>
    /// Tests if two AABB overlap with an Edge Condition, using vectorized instructions.<br/>Optimized for a *single* overlap, if you are testing one AABB against a set of AABBs, see documentation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TestOverlapWithRule(RectangleF rectA, RectangleF rectB, EdgeCollisionRule rule) {
        var a = rectA.Wind(); // Inlined
        var b = rectB.AsVector128(); // No operation. Just makes the C# compiler treat it as the right data
        var mask = VectorUtils.RuleToMask(rule); // Inlined 
        var res = Vector128.GreaterThan(a, b).AsInt32() | (Vector128.Equals(a.AsInt32(), b.AsInt32()) & mask);
        return Vector128.EqualsAll(res, Vector128<int>.AllBitsSet);
    }
}
