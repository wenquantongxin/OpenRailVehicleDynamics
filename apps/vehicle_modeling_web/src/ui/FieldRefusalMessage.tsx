import { useEffect, useRef } from 'react';

// What a field says under its inputs when the service refused what was
// entered: the service's reason, scrolled into view when it appears.

export function FieldRefusalMessage({ message }: { message: string }) {
  const shown = useRef<HTMLParagraphElement>(null);
  useEffect(() => {
    shown.current?.scrollIntoView({ block: 'nearest' });
  }, [message]);
  return (
    <p className="field-message refused" role="alert" ref={shown}>
      {message}
    </p>
  );
}
