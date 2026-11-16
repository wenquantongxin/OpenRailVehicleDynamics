import { Fragment } from 'react';

/**
 * An element name as it is, since other properties refer to it verbatim,
 * allowed to wrap after its underscores and dots.
 */
export function WrappableElementName({ name }: { name: string }) {
  const pieces = name.split(/(?<=[_.])/);
  return (
    <>
      {pieces.map((piece, index) => (
        <Fragment key={index}>
          {index > 0 && <wbr />}
          {piece}
        </Fragment>
      ))}
    </>
  );
}
