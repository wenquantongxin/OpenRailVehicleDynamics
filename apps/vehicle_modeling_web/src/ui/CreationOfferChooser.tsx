import type { CreationOffer, ElementKindDescription } from '../service/vehicle_modeling_service_protocol.ts';
import { BilingualTitleText } from './BilingualTitleText.tsx';
import { elementKindTitle } from './find_element_kind_title.ts';

// What a row's "+" asks first when it offers several kinds: each kind by its
// title, in the order the row offers them. The chosen kind's form follows.

export function CreationOfferChooser(props: { kinds: readonly ElementKindDescription[]; offers: readonly CreationOffer[]; onChoose: (offer: CreationOffer) => void; onCancel: () => void }) {
  return (
    <div className="creation-form">
      <div className="property-list">
        <div className="chooser-choices">
          {props.offers.map((offer) => (
            <button type="button" key={offer.kind} className="chooser-choice" onClick={() => props.onChoose(offer)}>
              <BilingualTitleText title={elementKindTitle(props.kinds, offer.kind)} />
            </button>
          ))}
        </div>
      </div>
      <div className="form-actions">
        <div className="form-buttons">
          <button type="button" className="chip" onClick={props.onCancel}>
            <span className="en">Cancel</span>
            <span className="zh">取消</span>
          </button>
        </div>
      </div>
    </div>
  );
}
