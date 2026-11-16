// What a number field says under its inputs when a text is no number.

export function UnreadableNumberMessage() {
  return (
    <p className="field-message">
      Enter a decimal number, e.g. 2.5e5 <span className="zh">请输入十进制数</span>
    </p>
  );
}
