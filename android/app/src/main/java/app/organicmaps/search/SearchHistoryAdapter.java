package app.organicmaps.search;

import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import androidx.annotation.NonNull;
import androidx.recyclerview.widget.RecyclerView;
import app.organicmaps.MwmApplication;
import app.organicmaps.R;
import app.organicmaps.sdk.routing.RoutingController;
import app.organicmaps.sdk.search.SearchRecents;
import app.organicmaps.util.Graphics;
import app.organicmaps.widget.SearchToolbarController;
import com.google.android.material.dialog.MaterialAlertDialogBuilder;

class SearchHistoryAdapter extends RecyclerView.Adapter<SearchHistoryAdapter.ViewHolder>
{
  private static final int TYPE_ITEM = 0;
  private static final int TYPE_CLEAR = 1;
  private static final int TYPE_MY_POSITION = 2;

  @NonNull
  private final SearchToolbarController mSearchToolbarController;
  private final boolean mShowMyPosition;

  public static class ViewHolder extends RecyclerView.ViewHolder
  {
    private final TextView mText;

    public ViewHolder(View itemView)
    {
      super(itemView);
      // A recent query has a button to delete it next to the text, other items are just a text.
      mText = itemView instanceof TextView ? (TextView) itemView : itemView.findViewById(R.id.text);
      Graphics.tint(mText);
    }
  }

  public SearchHistoryAdapter(@NonNull SearchToolbarController searchToolbarController, boolean showMyPosition)
  {
    SearchRecents.refresh();
    mSearchToolbarController = searchToolbarController;
    mShowMyPosition = showMyPosition;
  }

  @Override
  public ViewHolder onCreateViewHolder(ViewGroup viewGroup, int type)
  {
    final ViewHolder res;

    switch (type)
    {
    case TYPE_ITEM:
      res = new ViewHolder(
          LayoutInflater.from(viewGroup.getContext()).inflate(R.layout.item_search_recent, viewGroup, false));
      res.mText.setOnClickListener(v -> mSearchToolbarController.setQuery(res.mText.getText()));
      res.itemView.findViewById(R.id.delete).setOnClickListener(v -> askToDelete(v, res.mText.getText().toString()));
      break;

    case TYPE_CLEAR:
      res = new ViewHolder(
          LayoutInflater.from(viewGroup.getContext()).inflate(R.layout.item_search_clear_history, viewGroup, false));
      res.mText.setOnClickListener(v -> {
        SearchRecents.clear();
        notifyDataSetChanged();
      });
      break;

    case TYPE_MY_POSITION:
      res = new ViewHolder(
          LayoutInflater.from(viewGroup.getContext()).inflate(R.layout.item_search_my_position, viewGroup, false));
      res.mText.setOnClickListener(v -> {
        RoutingController.get().onPoiSelected(
            MwmApplication.from(viewGroup.getContext()).getLocationHelper().getMyPosition());
        mSearchToolbarController.onUpClick();
      });
      break;

    default: throw new IllegalArgumentException("Unsupported ViewHolder type given");
    }

    Graphics.tint(res.mText);
    return res;
  }

  private void askToDelete(@NonNull View view, @NonNull String query)
  {
    new MaterialAlertDialogBuilder(view.getContext(), R.style.MwmTheme_AlertDialog)
        .setTitle(R.string.nogps_delete_recent_query)
        .setMessage(query)
        .setPositiveButton(R.string.yes,
                           (dialog, which) -> {
                             SearchRecents.remove(query);
                             notifyDataSetChanged();
                           })
        .setNegativeButton(R.string.no, null)
        .show();
  }

  @Override
  public void onBindViewHolder(ViewHolder viewHolder, int position)
  {
    if (getItemViewType(position) == TYPE_ITEM)
    {
      if (mShowMyPosition)
        position--;

      viewHolder.mText.setText(SearchRecents.get(position));
    }
  }

  @Override
  public int getItemCount()
  {
    int res = SearchRecents.getSize();
    if (res > 0)
      res++;

    if (mShowMyPosition)
      res++;

    return res;
  }

  @Override
  public int getItemViewType(int position)
  {
    if (mShowMyPosition)
    {
      if (position == 0)
        return TYPE_MY_POSITION;

      position--;
    }

    return (position < SearchRecents.getSize() ? TYPE_ITEM : TYPE_CLEAR);
  }
}
