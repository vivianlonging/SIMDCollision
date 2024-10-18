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
        set => SetLeft(value);
    }
    public float Top {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => top;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => SetTop(value);
    }
    public float Right {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => -nRight;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => SetRight(value);
    }
    public float Bottom {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => -nBottom;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => SetBottom(value);
    }
    public float X {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => left;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => SetLeft(value);
    }
    public float Y {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => top;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => SetTop(value);
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetLeft(float value, bool MoveRectangleWithChange = true) {
        float diff = value - left;
        if (MoveRectangleWithChange) nRight -= diff;
        left += diff;
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetTop(float value, bool MoveRectangleWithChange = true) {
        float diff = value - top;
        if (MoveRectangleWithChange) nBottom -= diff;
        top += diff;
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetRight(float value, bool MoveRectangleWithChange = true) {
        float diff = nRight + value; // value - right
        if (MoveRectangleWithChange) left += diff;
        nRight -= diff;
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetBottom(float value, bool MoveRectangleWithChange = true) {
        float diff = nBottom + value; // value - bottom
        if (MoveRectangleWithChange) top += diff;
        nBottom -= diff;
    }
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
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static RectangleF Union(RectangleF a, RectangleF b) => VectorUtils.AsRectangleF(Vector128.Min(a.AsVector128(), b.AsVector128()));
    /// <summary>
    /// Does <i><b>NOT</b></i> represent whether or not the two rectangles overlap. Returns the intersection of two rectangles.<br/>
    /// max(a.Left, b.Left), max(a.Top, b.Top), min(a.Right, b.Right), min(a.Bottom, b.Bottom)
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static RectangleF Intersection(RectangleF a, RectangleF b) => VectorUtils.AsRectangleF(Vector128.Max(a.AsVector128(), b.AsVector128()));

    /// <summary>
    /// Tests if two AABB overlap.
    /// </summary>
    /// <param name="a">Rectangle A</param>
    /// <param name="b">Rectangle B</param>
    /// <returns>Whether or not the two rectangles overlapped</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TestOverlap(RectangleF a, RectangleF b) => TestOverlap(a.AsVector128(), b.AsVector128());
    /// <summary>
    /// Tests if two AABB overlap, using vectorized instructions.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TestOverlap(Vector128<float> rectA, Vector128<float> rectB) {
        rectA = Vector128.Shuffle(-rectA, Vector128.Create(2, 3, 0, 1));
        return Vector128.GreaterThanOrEqualAll(rectA, rectB);
    }
    /// <summary>
    /// Tests if two AABB overlap with an Edge Condition, using Vectorized instructions.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TestOverlapWithRule(RectangleF rectA, RectangleF rectB, EdgeCollisionRule rule) {
        var a = rectA.Wind();
        var b = rectB.AsVector128();
        var mask = VectorUtils.RuleToMask(rule);
        var res = Vector128.GreaterThan(a, b).AsInt32() | (Vector128.Equals(a.AsInt32(), b.AsInt32()) & mask);
        return Vector128.EqualsAll(res, Vector128<int>.AllBitsSet);
    }
}
