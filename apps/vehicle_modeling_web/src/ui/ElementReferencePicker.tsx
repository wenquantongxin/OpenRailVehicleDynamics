import { useEffect, useId, useState } from 'react';

import { elementKey, sameElement, type ElementReference } from '../service/vehicle_modeling_service_protocol.ts';
import { choicesShowKind, referenceChoices, type ReferenceDescription } from './element_reference_picker_rules.ts';
import { identifierLabel } from './identifier_display_labels.ts';
import { WrappableElementName } from './WrappableElementName.tsx';

// A reference field: the name of an element the property may name, chosen
// from the outline. Focusing it lists the choices; typing narrows them; the
// arrow keys move through them and Enter takes one; Escape, or leaving the
// field, takes none. Where the property may name more than one kind, every
// choice shows its kind; where it is optional, none is a choice.

export function ElementReferencePicker(props: {
  description: ReferenceDescription;
  elements: readonly ElementReference[];
  /** The name held; null names none; undefined where a creation form has no choice yet. */
  value: string | null | undefined;
  disabled: boolean;
  refused: boolean;
  onChoose: (name: string | null) => void;
}) {
  const { description, elements, value } = props;
  const [open, setOpen] = useState(false);
  const [filter, setFilter] = useState('');
  const [active, setActive] = useState(0);
  const listId = useId();
  const choices = open ? referenceChoices(elements, description, filter) : [];
  const showKind = choicesShowKind(description);
  const named = value === null || value === undefined ? undefined : elements.find((element) => description.referencedKinds.includes(element.kind) && element.name === value);

  useEffect(() => {
    if (open) document.getElementById(`${listId}-${active}`)?.scrollIntoView({ block: 'nearest' });
  }, [open, active, listId]);

  const close = (): void => {
    setOpen(false);
    setFilter('');
    setActive(0);
  };
  const choose = (choice: ElementReference | null): void => {
    close();
    const name = choice === null ? null : choice.name;
    if (name !== value) props.onChoose(name);
  };
  const isHeld = (choice: ElementReference | null): boolean => (choice === null ? value === null : named !== undefined && sameElement(choice, named));
  const kindShown = !open && showKind && named !== undefined;

  return (
    <div
      className={`reference-picker ${open ? 'open' : ''}`}
      onBlur={(event) => {
        if (!event.currentTarget.contains(event.relatedTarget as Node | null)) close();
      }}
    >
      <div className={`reference-input ${kindShown ? 'with-kind' : ''}`}>
        <input
          type="text"
          role="combobox"
          aria-label={identifierLabel(description.name)}
          aria-expanded={open}
          aria-controls={listId}
          aria-autocomplete="list"
          aria-activedescendant={open && choices.length > 0 ? `${listId}-${active}` : undefined}
          aria-invalid={props.refused}
          spellCheck={false}
          autoComplete="off"
          readOnly={props.disabled}
          value={open ? filter : (value ?? '')}
          placeholder={open ? 'Type to filter · 输入以筛选' : value === null ? 'none · 无' : 'Choose · 选择'}
          onFocus={() => setOpen(!props.disabled)}
          onClick={() => setOpen(!props.disabled)}
          onChange={(event) => {
            setOpen(true);
            setFilter(event.target.value);
            setActive(0);
          }}
          onKeyDown={(event) => {
            if (event.key === 'ArrowDown') {
              event.preventDefault();
              setOpen(!props.disabled);
              setActive((index) => Math.min(index + 1, Math.max(choices.length - 1, 0)));
            } else if (event.key === 'ArrowUp') {
              event.preventDefault();
              setActive((index) => Math.max(index - 1, 0));
            } else if (event.key === 'Enter' && open) {
              // Taken here, not by a form around the field.
              event.preventDefault();
              if (choices.length > 0) choose(choices[Math.min(active, choices.length - 1)]!);
            } else if (event.key === 'Escape' && open) {
              event.stopPropagation();
              close();
            }
          }}
        />
        {kindShown && <span className="reference-kind">{identifierLabel(named.kind)}</span>}
      </div>
      {open && (
        <ul className="reference-choices" role="listbox" id={listId}>
          {choices.length === 0 && (
            <li className="reference-empty">
              Nothing matches <span className="zh">无匹配项</span>
            </li>
          )}
          {choices.map((choice, index) => (
            <li
              key={choice === null ? 'none' : elementKey(choice)}
              id={`${listId}-${index}`}
              role="option"
              aria-selected={index === active}
              className={`${index === active ? 'active' : ''} ${isHeld(choice) ? 'held' : ''}`}
              // A press must not take focus from the input before the choice is made.
              onMouseDown={(event) => event.preventDefault()}
              onMouseEnter={() => setActive(index)}
              onClick={() => choose(choice)}
            >
              {choice === null ? (
                <span className="choice-none">
                  none <span className="zh">无</span>
                </span>
              ) : (
                <>
                  <span className="choice-name">
                    <WrappableElementName name={choice.name} />
                  </span>
                  {showKind && <span className="choice-kind">{identifierLabel(choice.kind)}</span>}
                </>
              )}
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
